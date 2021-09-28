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

#include "assisted_teleop.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_recoveries
{

AssistedTeleop::AssistedTeleop()
: Recovery<AssistedTeleopAction>(),
  feedback_(std::make_shared<AssistedTeleopAction::Feedback>())
{
}

AssistedTeleop::~AssistedTeleop()
{
}

void AssistedTeleop::onConfigure()
{
  auto node = node_.lock();
  logger_ = node->get_logger();

  vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::SystemDefaultsQoS(),
    std::bind(
      &AssistedTeleop::vel_callback,
      this, std::placeholders::_1));

  vel_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

  costmap_sub_ = std::make_unique<nav2_costmap_2d::CostmapSubscriber>(
    node_, "local_costmap/costmap_raw");

  nav2_util::declare_parameter_if_not_declared(
    node,
    "projection_time", rclcpp::ParameterValue(1.0));
  node->get_parameter("projection_time", projection_time_);
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

void
AssistedTeleop::vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  speed_x = msg->linear.x;
  speed_y = msg->linear.y;
  angular_vel_ = msg->angular.z;

  costmap_ros_ = costmap_sub_->getCostmap();
  if (go && speed_x != 0) {
    if (!checkCollision(scaling_factor)) {
      RCLCPP_INFO(logger_, "Reducing velocity by %.2f", scaling_factor);
      move = true;
    }
  }
}

bool
AssistedTeleop::checkCollision(double & scaling_factor)
{
  const double dt = costmap_ros_->getResolution() / std::fabs(speed_x);
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

void
AssistedTeleop::onCleanup()
{
  vel_sub_.reset();
  RCLCPP_INFO(logger_, "Cleaning up velocity subscriber");
}

}  // namespace nav2_recoveries

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_recoveries::AssistedTeleop, nav2_core::Recovery)
