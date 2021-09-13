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

#ifndef NAV2_RECOVERIES__PLUGINS__AssistedTeleop_HPP_
#define NAV2_RECOVERIES__PLUGINS__AssistedTeleop_HPP_

#include <chrono>
#include <string>
#include <memory>

#include "nav2_recoveries/recovery.hpp"
#include "nav2_msgs/action/assisted_teleop.hpp"

namespace nav2_recoveries
{
using AssistedTeleopAction = nav2_msgs::action::AssistedTeleop;

/**
 * @class nav2_recoveries::AssistedTeleop
 * @brief An action server recovery for AssistedTeleoping a fixed duration
 */
class AssistedTeleop : public Recovery<AssistedTeleopAction>
{
public:
  /**
   * @brief A constructor for nav2_recoveries::AssistedTeleop
   */
  AssistedTeleop();
  ~AssistedTeleop();

  /**
   * @brief Initialization to run behavior
   * @param command Goal to execute
   * @return Status of recovery
   */
  Status onRun(const std::shared_ptr<const AssistedTeleopAction::Goal> command) override;

  /**
   * @brief Loop function to run behavior
   * @return Status of recovery
   */
  Status onCycleUpdate() override;

  void onCleanup() override;
 
protected:
  std::chrono::time_point<std::chrono::steady_clock> AssistedTeleop_end_;
  AssistedTeleopAction::Feedback::SharedPtr feedback_;

  bool projectPose();
  void vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  bool isCollisionFree(
  geometry_msgs::msg::Pose2D & pose2d);
  void computeVelocity(bool isCollisionFree);
  double min_rotational_vel_;
  double max_rotational_vel_;
  double rotational_acc_lim_;
  double speed_ = 0;
  double prev_yaw_;
  double relative_yaw_;
  double simulate_ahead_time_;
  double projection_time = 5.0;
  geometry_msgs::msg::PoseStamped current_pose;
  geometry_msgs::msg::Pose2D projected_pose;
  // geometry_msgs::msg::Twist::SharedPtr cmd_vel_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
  std::string vel_topic_ = "cmd_vel";


};

}  // namespace nav2_recoveries

#endif  // NAV2_RECOVERIES__PLUGINS__AssistedTeleop_HPP_
