// Copyright (c) 2021 Anushree Sabnis
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>
#include <chrono>
#include <memory>
#include <utility>
#include <string>

#include "assisted_teleop.hpp"
#include "nav2_msgs/action/assisted_teleop.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_recoveries
{

void AssistedTeleop::onConfigure()
{
  auto node = node_.lock();
  logger_ = node->get_logger();

  nav2_util::declare_parameter_if_not_declared(
    node,
    "projection_time", rclcpp::ParameterValue(1.0));
  node->get_parameter("projection_time", projection_time_);

  nav2_util::declare_parameter_if_not_declared(
    node,
    "linear_velocity_threshold_", rclcpp::ParameterValue(0.06));
  node->get_parameter("linear_velocity_threshold_", linear_velocity_threshold_);

  nav2_util::declare_parameter_if_not_declared(
    node,
    "cmd_vel_topic", rclcpp::ParameterValue(std::string("cmd_vel_topic")));
  node->get_parameter("cmd_vel_topic", cmd_vel_topic_);

  vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::SystemDefaultsQoS(),
    std::bind(
      &AssistedTeleop::vel_callback,
      this, std::placeholders::_1));

  vel_pub_ = node->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 1);

  costmap_sub_ = std::make_unique<nav2_costmap_2d::CostmapSubscriber>(
    node_, "local_costmap/costmap_raw");
}

void
AssistedTeleop::vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  speed_x = msg->linear.x;
  speed_y = msg->linear.y;
  angular_vel_ = msg->angular.z;
  if (go) {
    if (!checkCollision()) {
      moveRobot();
    }
  }
}

bool
AssistedTeleop::updatePose()
{
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "Current robot pose is not available.");
    return false;
  }
  projected_pose.x = current_pose.pose.position.x;
  projected_pose.y = current_pose.pose.position.y;
  projected_pose.theta = tf2::getYaw(
    current_pose.pose.orientation);
  return true;
}

void
AssistedTeleop::projectPose(
  double speed_x, double speed_y,
  double angular_vel_, double projection_time)
{
  // Project Pose by time increment
  projected_pose.x += projection_time * (
    speed_x * cos(projected_pose.theta));
  projected_pose.y += projection_time * (
    speed_y * sin(projected_pose.theta));
  projected_pose.theta += projection_time *
    angular_vel_;
}

bool
AssistedTeleop::checkCollision()
{
  const double dt =
    (speed_x != 0) ? (costmap_ros_->getResolution() / std::fabs(speed_x)) : projection_time_;
  int loopcount = 1;

  while (true) {
    if (updatePose()) {
      double time_to_collision = loopcount * dt;
      if (time_to_collision >= projection_time_) {
        scaling_factor = 1;
        break;
      }
      scaling_factor = projection_time_ / (time_to_collision);
      loopcount++;

      projectPose(speed_x, speed_y, angular_vel_, time_to_collision);

      if (!collision_checker_->isCollisionFree(projected_pose)) {
        RCLCPP_WARN(logger_, "Collision approaching in %.2f seconds", time_to_collision);
        return false;
      }
    }
  }
  return true;
}

// Stop the robot with a scaled-down velocity
void
AssistedTeleop::moveRobot()
{
  RCLCPP_INFO(logger_, "scaling_factor is %.2f", scaling_factor);
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();

  cmd_vel->linear.x = speed_x / scaling_factor;
  cmd_vel->linear.y = speed_y;
  cmd_vel->angular.z = angular_vel_ / scaling_factor;

  if (cmd_vel->linear.x < linear_velocity_threshold_) {
    stopRobot();
  }

  vel_pub_->publish(std::move(cmd_vel));
}

}  // namespace nav2_recoveries

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_recoveries::AssistedTeleop, nav2_core::Recovery)
