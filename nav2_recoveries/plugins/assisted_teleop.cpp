// Copyright (c) 2019 Samsung Research America
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

#include "assisted_teleop.hpp"
 
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
  AssistedTeleop_end_ = std::chrono::steady_clock::now() +
    rclcpp::Duration(command->time).to_chrono<std::chrono::nanoseconds>();
  vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
  vel_topic_, rclcpp::SystemDefaultsQoS(),
  std::bind(&AssistedTeleop::vel_callback, this, std::placeholders::_1));

  

  return Status::SUCCEEDED;
}

Status AssistedTeleop::onCycleUpdate()
{
  auto current_point = std::chrono::steady_clock::now();
  auto time_left =
    std::chrono::duration_cast<std::chrono::nanoseconds>(AssistedTeleop_end_ - current_point).count();

  feedback_->time_left = rclcpp::Duration(rclcpp::Duration::from_nanoseconds(time_left));
  action_server_->publish_feedback(feedback_);

  if (time_left > 0) {
    if (projectPose())
    {
      std::cout << "col checker : " << isCollisionFree(projected_pose) << std::endl;
    }
    return Status::RUNNING;
  } else {
    return Status::SUCCEEDED;
  }
}

bool AssistedTeleop::projectPose()
{
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "Current robot pose is not available.");
    return false;
  }

  prev_yaw_ = tf2::getYaw(current_pose.pose.orientation);

  
  projected_pose.x = current_pose.pose.position.x + projection_time *
        (speed_ * (cos(prev_yaw_)));
  projected_pose.y = current_pose.pose.position.y + projection_time *
        (speed_ * (sin(prev_yaw_)));
  // projected_pose.theta = prev_yaw_ + ;

  return true;
  // relative_yaw_ = 0.0;
}

void AssistedTeleop::vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  speed_ = msg->linear.x;
  // std::cout << "cmd vel : " << cmd_vel_ << std::endl;
}

bool AssistedTeleop::isCollisionFree(
  geometry_msgs::msg::Pose2D & pose2d)
{
  if (!collision_checker_->isCollisionFree(pose2d)) {
      RCLCPP_WARN(logger_,"Projected footprint is not Collision Free");
      return false;
    }
  else
    return true;
}

void AssistedTeleop::computeVelocity(bool isCollisionFree)
{
  if (!isCollisionFree)
  {
    RCLCPP_WARN(logger_,"Stopping ROBOT");
    stopRobot();
  }
}

void AssistedTeleop::onCleanup()
{
  vel_sub_.reset();
  RCLCPP_INFO(logger_, "Cleaning up velocity subscriber");
  // return Status::SUCCEEDED;
}

}  // namespace nav2_recoveries

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_recoveries::AssistedTeleop, nav2_core::Recovery)
