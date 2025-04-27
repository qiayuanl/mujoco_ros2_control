// Copyright (c) 2025 Sangtaek Lee
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <memory>

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/system_interface.hpp"

#include "mujoco_ros2_control/mujoco_ros2_control.hpp"

namespace mujoco_ros2_control
{
MujocoRos2Control::MujocoRos2Control(
  rclcpp::Node::SharedPtr &node, mjModel *mujoco_model, mjData *mujoco_data)
    : node_(node),
      mj_model_(mujoco_model),
      mj_data_(mujoco_data),
      logger_(rclcpp::get_logger(node_->get_name() + std::string(".mujoco_ros2_control"))),
      stop_cm_thread_(false),
      control_period_(rclcpp::Duration(1, 0)),
      last_update_sim_time_ros_(0, 0, RCL_ROS_TIME)
{
}

MujocoRos2Control::~MujocoRos2Control()
{
  stop_cm_thread_ = true;
  cm_executor_->remove_node(controller_manager_);
  cm_executor_->cancel();
  cm_thread_.join();
}

void MujocoRos2Control::init()
{
  clock_publisher_ = node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

  for (int i = 0; i < mj_model_->nsensor; i++)
  {
    std::string sensor_site = mj_id2name(mj_model_, mjOBJ_SITE, mj_model_->sensor_objid[i]);
    if (
      mj_model_->sensor_type[i] == mjSENS_FRAMEPOS || mj_model_->sensor_type[i] == mjSENS_FRAMEQUAT)
    {
      if (odom_publishers_.find(sensor_site) == odom_publishers_.end())
      {
        odom_publishers_.insert(
          std::make_pair(
            sensor_site, node_->create_publisher<nav_msgs::msg::Odometry>(sensor_site, 10)));
        odom_msgs_.insert(std::make_pair(sensor_site, nav_msgs::msg::Odometry()));
        RCLCPP_INFO_STREAM(logger_, "Setting up publisher for position sensor: " << sensor_site);
      }
    }
  }

  // Read urdf from ros parameter server then
  // setup actuators and mechanism control node.
  std::string urdf_string;
  std::vector<hardware_interface::HardwareInfo> control_hardware_info;
  try
  {
    node_->declare_parameter("robot_description", "");
    urdf_string = node_->get_parameter("robot_description").as_string();
    control_hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf_string);
  }
  catch (const std::runtime_error &ex)
  {
    RCLCPP_ERROR_STREAM(logger_, "Error parsing URDF : " << ex.what());
    return;
  }

  try
  {
    robot_hw_sim_loader_ = std::make_shared<pluginlib::ClassLoader<MujocoSystemInterface>>(
      "mujoco_ros2_control", "mujoco_ros2_control::MujocoSystemInterface");
  }
  catch (pluginlib::LibraryLoadException &ex)
  {
    RCLCPP_ERROR_STREAM(logger_, "Failed to create hardware interface loader:  " << ex.what());
    return;
  }

  std::unique_ptr<hardware_interface::ResourceManager> resource_manager =
    std::make_unique<hardware_interface::ResourceManager>();

  try
  {
    resource_manager->load_urdf(urdf_string, false, false);
  }
  catch (...)
  {
    RCLCPP_ERROR(logger_, "Error while initializing URDF!");
  }

  for (const auto &hardware : control_hardware_info)
  {
    std::string robot_hw_sim_type_str_ = hardware.hardware_class_type;
    std::unique_ptr<MujocoSystemInterface> mujoco_system;
    try
    {
      mujoco_system = std::unique_ptr<MujocoSystemInterface>(
        robot_hw_sim_loader_->createUnmanagedInstance(robot_hw_sim_type_str_));
    }
    catch (pluginlib::PluginlibException &ex)
    {
      RCLCPP_ERROR_STREAM(logger_, "The plugin failed to load. Error: " << ex.what());
      continue;
    }

    urdf::Model urdf_model;
    urdf_model.initString(urdf_string);
    if (!mujoco_system->init_sim(node_, mj_model_, mj_data_, urdf_model, hardware))
    {
      RCLCPP_FATAL(logger_, "Could not initialize robot simulation interface");
      return;
    }

    resource_manager->import_component(std::move(mujoco_system), hardware);

    rclcpp_lifecycle::State state(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      hardware_interface::lifecycle_state_names::ACTIVE);
    resource_manager->set_component_state(hardware.name, state);
  }

  // Create the controller manager
  RCLCPP_INFO(logger_, "Loading controller_manager");
  cm_executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  controller_manager_ = std::make_shared<controller_manager::ControllerManager>(
    std::move(resource_manager), cm_executor_, "controller_manager", node_->get_namespace());

  cm_executor_->add_node(controller_manager_);

  if (!controller_manager_->has_parameter("update_rate"))
  {
    RCLCPP_ERROR_STREAM(logger_, "controller manager doesn't have an update_rate parameter");
    return;
  }

  auto update_rate = controller_manager_->get_parameter("update_rate").as_int();
  control_period_ = rclcpp::Duration(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / static_cast<double>(update_rate))));

  // Force setting of use_sime_time parameter
  controller_manager_->set_parameter(
    rclcpp::Parameter("use_sim_time", rclcpp::ParameterValue(true)));

  stop_cm_thread_ = false;
  auto spin = [this]()
  {
    while (rclcpp::ok() && !stop_cm_thread_)
    {
      cm_executor_->spin_once();
    }
  };
  cm_thread_ = std::thread(spin);
}

void MujocoRos2Control::update()
{
  // Get the simulation time and period
  auto sim_time = mj_data_->time;
  int sim_time_sec = static_cast<int>(sim_time);
  int sim_time_nanosec = static_cast<int>((sim_time - sim_time_sec) * 1000000000);

  rclcpp::Time sim_time_ros(sim_time_sec, sim_time_nanosec, RCL_ROS_TIME);
  rclcpp::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;

  publish_sim_time(sim_time_ros);
  publish_poses(sim_time_ros);

  if (sim_period.seconds() < 0)
  {
    RCLCPP_INFO(logger_, "Simulation time reset.");
    last_update_sim_time_ros_ = sim_time_ros;
  }

  if (sim_period >= control_period_)
  {
    controller_manager_->read(sim_time_ros, sim_period);
    controller_manager_->update(sim_time_ros, sim_period);
    // use same time as for read and update call - this is how it is done in ros2_control_node
    controller_manager_->write(sim_time_ros, sim_period);
    last_update_sim_time_ros_ = sim_time_ros;
  }
}

void MujocoRos2Control::publish_sim_time(rclcpp::Time sim_time)
{
  // TODO(sangteak601)
  rosgraph_msgs::msg::Clock sim_time_msg;
  sim_time_msg.clock = sim_time;
  clock_publisher_->publish(sim_time_msg);
}

void MujocoRos2Control::publish_poses(rclcpp::Time sim_time)
{
  int index = 0;
  for (int i = 0; i < mj_model_->nsensor; i++)
  {
    std::string sensor_site = mj_id2name(mj_model_, mjOBJ_SITE, mj_model_->sensor_objid[i]);
    auto &pose_msg = odom_msgs_.at(sensor_site);
    pose_msg.header.stamp = sim_time;
    pose_msg.header.frame_id = "odom";
    if (mj_model_->sensor_type[i] == mjSENS_FRAMEPOS)
    {
      pose_msg.pose.pose.position.x = mj_data_->sensordata[index + 0];
      pose_msg.pose.pose.position.y = mj_data_->sensordata[index + 1];
      pose_msg.pose.pose.position.z = mj_data_->sensordata[index + 2];
    }
    else if (mj_model_->sensor_type[i] == mjSENS_FRAMEQUAT)
    {
      pose_msg.pose.pose.orientation.w = mj_data_->sensordata[index + 0];
      pose_msg.pose.pose.orientation.x = mj_data_->sensordata[index + 1];
      pose_msg.pose.pose.orientation.y = mj_data_->sensordata[index + 2];
      pose_msg.pose.pose.orientation.z = mj_data_->sensordata[index + 3];
    }
    index += mj_model_->sensor_dim[i];
  }
  for (const auto &pose_publisher : odom_publishers_)
  {
    pose_publisher.second->publish(odom_msgs_.at(pose_publisher.first));
  }
}

}  // namespace mujoco_ros2_control
