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

Status AssistedTeleop::onRun(const std::shared_ptr<const AssistedTeleopAction::Goal> command)
{
  auto node = node_.lock();
  logger_ = node->get_logger();
  assisted_teleop_end_ = std::chrono::steady_clock::now() +
    rclcpp::Duration(command->time).to_chrono<std::chrono::nanoseconds>();

  vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::SystemDefaultsQoS(),
    std::bind(
      &AssistedTeleop::vel_callback,
      this, std::placeholders::_1));

  costmap_sub_ = std::make_unique<nav2_costmap_2d::CostmapSubscriber>(
    node_, "local_costmap/costmap_raw");

  return Status::SUCCEEDED;
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
  // Project Pose
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
  cmd_vel_->linear.x = msg->linear.x;
  cmd_vel_->angular.z = msg->angular.z;

  costmap_ros_ = costmap_sub_->getCostmap();
  if (go && speed_x != 0) {
    if (updatePose()) {
      if (!checkCollision()) {
        moveRobot();
        RCLCPP_WARN(logger_, "Collision approaching in %.2f seconds", col_time);
      }
    }
  }
}

bool
AssistedTeleop::checkCollision()
{
  RCLCPP_INFO(logger_, "costmap res : %.2f ", costmap_ros_->getResolution());

  const double dt = costmap_ros_->getResolution() / speed_x;
  RCLCPP_INFO(logger_, "dt is %.2f", dt);
  loopcount = 1;
  while (1) {
    col_time = loopcount * dt;
    RCLCPP_INFO(logger_, "Col time is %.2f", col_time);

    if (col_time >= projection_time) {
      return true;
    }
    loopcount++;
    projectPose(speed_x, speed_x, angular_vel_, col_time);
    if (!collision_checker_->isCollisionFree(projected_pose)) {
      return false;
    }
  }
  return true;
}

void
AssistedTeleop::moveRobot()
{
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  double mag = sqrt(
    cmd_vel_->linear.x * cmd_vel_->linear.x +
    /*cmd_vel_->linear.y * cmd_vel_->linear.y +*/
    cmd_vel_->angular.z * cmd_vel_->angular.z);
  int scaling_factor = projection_time / col_time;

  if (mag != 0.0) {
    cmd_vel->linear.x = cmd_vel_->linear.x / ( scaling_factor);
    cmd_vel->linear.y = cmd_vel_->linear.y;
    cmd_vel->angular.z = cmd_vel_->angular.z / ( scaling_factor);
  }
  vel_pub_->publish(std::move(cmd_vel));
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
