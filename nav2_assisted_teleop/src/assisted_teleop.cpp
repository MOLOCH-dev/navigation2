/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Author(s): Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#include <algorithm>
#include <string>
#include <memory>

#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_assisted_teleop/assisted_teleop.hpp"
#include "nav2_util/geometry_utils.hpp"

using std::hypot;
using std::min;
using std::max;
using std::abs;
using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using namespace nav2_costmap_2d;  // NOLINT

namespace nav2_assisted_teleop
{



void AssistedTeleopController::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
{
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();

  //double transform_tolerance = 0.1;
  //double control_frequency = 20.0;

}

void AssistedTeleopController::cleanup() {
  RCLCPP_INFO(
  logger_,
  "Cleaning up controller: %s of type"
  " nav2_assisted_teleop::AssistedTeleoptController",
  plugin_name_.c_str());
}

void AssistedTeleopController::activate(){
  RCLCPP_INFO(
  logger_,
  "Activating controller: %s of type "
  "nav2_assisted_teleop::AssistedTController",
  plugin_name_.c_str());
}

void AssistedTeleopController::deactivate(){
    RCLCPP_INFO(
  logger_,
  "Deactivating controller: %s of type "
  "nav2_assisted_teleop::AssistedTeleopController",
  plugin_name_.c_str());
}
geometry_msgs::msg::TwistStamped AssistedTeleopController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed)
{
  std::cout<< pose.pose.position.x << std::endl;
  std::cout<< speed.linear.x << std::endl;

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  cmd_vel.twist.linear.x = speed.linear.x;
  cmd_vel.twist.angular.z = speed.angular.z;
  return cmd_vel;
}


void AssistedTeleopController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
}


}  // namespace nav2_pure_pursuit_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(nav2_assisted_teleop::AssistedTeleopController, nav2_core::Controller)