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

#include "mujoco_ros2_control/mujoco_ros2_control.hpp"

namespace mujoco_ros2_control
{

class MujocoResourceManager : public hardware_interface::ResourceManager
{
public:
  MujocoResourceManager(rclcpp::Node::SharedPtr &node, mjModel *mj_model, mjData *mj_data)
      : ResourceManager(node->get_node_clock_interface(), node->get_node_logging_interface()),
        node_(node),
        mj_system_loader_("mujoco_ros2_control", "mujoco_ros2_control::MujocoSystemInterface"),
        logger_(node->get_logger().get_child("MujocoResourceManager"))
  {
    mj_model_ = mj_model;
    mj_data_ = mj_data;
    node_ = node;
  }

  MujocoResourceManager(const MujocoResourceManager &) = delete;

  bool load_and_initialize_components(const std::string &urdf, unsigned int update_rate) override
  {
    components_are_loaded_and_initialized_ = true;

    const auto hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf);

    for (const auto &individual_hardware_info : hardware_info)
    {
      std::string robot_hw_sim_type_str_ = individual_hardware_info.hardware_plugin_name;
      RCLCPP_DEBUG(logger_, "Load hardware interface %s ...", robot_hw_sim_type_str_.c_str());

      // Load hardware
      std::unique_ptr<MujocoSystemInterface> mjSimSystem;
      std::scoped_lock guard(resource_interfaces_lock_, claimed_command_interfaces_lock_);
      try
      {
        mjSimSystem = std::unique_ptr<MujocoSystemInterface>(
          mj_system_loader_.createUnmanagedInstance(robot_hw_sim_type_str_));
      }
      catch (pluginlib::PluginlibException &ex)
      {
        RCLCPP_ERROR_STREAM(logger_, "The plugin failed to load. Error: " << ex.what());
        continue;
      }

      // initialize simulation required resource from the hardware info.
      urdf::Model urdf_model;
      urdf_model.initString(urdf);
      if (!mjSimSystem->init_sim(node_, mj_model_, mj_data_, urdf_model, individual_hardware_info))
      {
        RCLCPP_FATAL(logger_, "Could not initialize robot simulation interface");
        components_are_loaded_and_initialized_ = false;
        break;
      }
      RCLCPP_DEBUG(logger_, "Initialized hardware interface %s !", robot_hw_sim_type_str_.c_str());
      import_component(std::move(mjSimSystem), individual_hardware_info);
    }
    return components_are_loaded_and_initialized_;
  }

private:
  mjModel *mj_model_;
  mjData *mj_data_;
  std::shared_ptr<rclcpp::Node> node_;
  pluginlib::ClassLoader<MujocoSystemInterface> mj_system_loader_;

  rclcpp::Logger logger_;
};

MujocoRos2Control::MujocoRos2Control(
  rclcpp::Node::SharedPtr &node, rclcpp::NodeOptions cm_node_option, mjModel *mujoco_model,
  mjData *mujoco_data)
    : node_(node),
      cm_node_option_(cm_node_option),
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

  std::unique_ptr<hardware_interface::ResourceManager> resource_manager =
    std::make_unique<MujocoResourceManager>(node_, mj_model_, mj_data_);

  // Create the controller manager
  RCLCPP_INFO(logger_, "Loading controller_manager");
  cm_executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  controller_manager_ = std::make_shared<controller_manager::ControllerManager>(
    std::move(resource_manager), cm_executor_, "controller_manager", node_->get_namespace(),
    cm_node_option_);

  cm_executor_->add_node(node_);
  cm_executor_->add_node(controller_manager_);


  control_period_ = rclcpp::Duration(
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(
      1.0 / static_cast<double>(controller_manager_->get_update_rate()))));

  if (
    rcl_enable_ros_time_override(controller_manager_->get_trigger_clock()->get_clock_handle()) !=
    RCL_RET_OK)
  {
    RCLCPP_FATAL(logger_, "Failed to enable ROS time override: %s", rcl_get_error_string().str);
    rcl_reset_error();
    throw std::runtime_error("rcl_enable_ros_time_override failed");
  }

  stop_cm_thread_ = false;
  auto spin = [this]()
  {
    while (rclcpp::ok() && !stop_cm_thread_)
    {
      cm_executor_->spin_once();
    }
  };
  cm_thread_ = std::thread(spin);

  // Waiting RM to be initialized through topic robot_description
  while (!controller_manager_->is_resource_manager_initialized())
  {
    RCLCPP_WARN(logger_, "Waiting RM to load and initialize hardware...");
    std::this_thread::sleep_for(std::chrono::microseconds(100000));
  }
}

void MujocoRos2Control::update()
{
  // Get the simulation time and period
  auto sim_time = mj_data_->time;
  int sim_time_sec = static_cast<int>(sim_time);
  int sim_time_nanosec = static_cast<int>((sim_time - sim_time_sec) * 1000000000);

  rclcpp::Time sim_time_ros(sim_time_sec, sim_time_nanosec, RCL_ROS_TIME);
  rclcpp::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;
  if (
    rcl_set_ros_time_override(
      controller_manager_->get_trigger_clock()->get_clock_handle(),
      static_cast<rcl_time_point_value_t>(sim_time * 1e9)) != RCL_RET_OK)
  {
    RCLCPP_ERROR(logger_, "rcl_set_ros_time_override failed: %s", rcl_get_error_string().str);
    rcl_reset_error();
  }

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
