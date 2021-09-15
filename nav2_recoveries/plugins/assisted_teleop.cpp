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
  assisted_teleop_end_ = std::chrono::steady_clock::now() +
    rclcpp::Duration(command->time).to_chrono<std::chrono::nanoseconds>();

  vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
    vel_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&AssistedTeleop::vel_callback,
      this, std::placeholders::_1));
  // projection_time_ = costmap_->getResolution();



  

  return Status::SUCCEEDED;
}

Status AssistedTeleop::onCycleUpdate()
{
  auto current_point = std::chrono::steady_clock::now();
  auto time_left =
    std::chrono::duration_cast<std::chrono::nanoseconds>(assisted_teleop_end_ - current_point).count();

  feedback_->time_left = rclcpp::Duration(rclcpp::Duration::from_nanoseconds(time_left));
  action_server_->publish_feedback(feedback_);

  if (time_left > 0) { 
    if (speed_ != 0 || angular_vel_ != 0)
    {
      if (updatePose())
      {
        projectPose(speed_, angular_vel_);
        RCLCPP_INFO(logger_,"Col check : %d", checkCollision());
        RCLCPP_WARN(logger_,"Time to col : %.2f", col_time);
        moveRobot();
      }
    }
    return Status::RUNNING;
  } 
  else {
    return Status::SUCCEEDED;
  }

}

bool AssistedTeleop::updatePose()
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

void AssistedTeleop::projectPose(double speed_, double angular_vel_) 
{
  // Project Pose
  projected_pose.x += projection_time * (speed_ * (cos(projected_pose.theta)));
  projected_pose.y += projection_time * (speed_ * (sin(projected_pose.theta)));
  projected_pose.theta += projection_time * angular_vel_;

}


void AssistedTeleop::vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  speed_ = msg->linear.x;
  angular_vel_ = msg->angular.z;
  cmd_vel_->linear.x = msg->linear.x;
  cmd_vel_->angular.z = msg->angular.z;
}

bool AssistedTeleop::checkCollision()
{

  col_time = 0;
  double dt = projection_time / cycle_frequency_; // projection time varies, should I static cast this as well
  const int max_cycle_count = static_cast<int>(dt * cycle_frequency_ );// simulate_ahead_time_);
  RCLCPP_INFO(logger_,"Checking collision with dt : %.2f and %d cycles",dt,max_cycle_count);

  for (int i=0; i<cycle_frequency_; ++i)
  {
    speed_ += (i * dt) * (cmd_vel_->linear.x / cycle_frequency_);
    angular_vel_ += (i * dt) * (cmd_vel_->angular.z / cycle_frequency_);
    projectPose(speed_, angular_vel_);
    col_time = i * dt;
    if (!collision_checker_->isCollisionFree(projected_pose))
    {
      // computeVelocity(col_time);
      scaling_factor = col_time / projection_time;
      return false;
    }
    speed_ = cmd_vel_->linear.x;
    angular_vel_ = cmd_vel_->angular.z;
  }
  return true;
}

void AssistedTeleop::computeVelocity(double scaling_time)
{
  scaling_factor = scaling_time / projection_time;
  cmd_vel_->linear.x *= scaling_factor;
  cmd_vel_->angular.z *= scaling_factor;
  vel_pub_->publish(std::move(cmd_vel_));

}

void AssistedTeleop::moveRobot()
{
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel->linear.x = cmd_vel_->linear.x * scaling_factor;
  cmd_vel->linear.y = 0.0;
  cmd_vel->angular.z = cmd_vel_->angular.z * scaling_factor;

  vel_pub_->publish(std::move(cmd_vel));
}

void AssistedTeleop::onCleanup()
{
  vel_sub_.reset();
  RCLCPP_INFO(logger_, "Cleaning up velocity subscriber");

}

}  // namespace nav2_recoveries

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_recoveries::AssistedTeleop, nav2_core::Recovery)
