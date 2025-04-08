/*
 * @file mujoco_ros2_control_plugin.cpp
 * @date 1/23/25
 * @brief
 *
 * @copyright Copyright (c) 2025 Ruixiang Du (rdu)
 */

#include "mujoco_ros2_control/mujoco_ros2_control_plugin.hpp"

namespace mujoco_ros2_control
{
void MujocoRos2ControlPlugin::Configure(
  rclcpp::Node::SharedPtr &node, rclcpp::NodeOptions /*cm_node_option*/, mjModel *model,
  mjData *data)
{
  control_ = std::make_unique<MujocoRos2Control>(node, model, data);
  control_->init();
  RCLCPP_INFO_STREAM(
    node->get_logger(), "Mujoco ros2 controller has been successfully initialized !");
}

void MujocoRos2ControlPlugin::Update(mjModel *model, mjData *data)
{
  // update logic
  control_->update();
}

}  // namespace mujoco_ros2_control

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  mujoco_ros2_control::MujocoRos2ControlPlugin, mujoco_sim_ros2::MujocoPhysicsPlugin)
