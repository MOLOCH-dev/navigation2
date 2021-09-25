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


#ifndef NAV2_RECOVERIES__PLUGINS__ASSISTED_TELEOP_HPP_
#define NAV2_RECOVERIES__PLUGINS__ASSISTED_TELEOP_HPP_

#include <chrono>
#include <string>
#include <memory>

#include "nav2_recoveries/assisted_teleop_server.hpp"
#include "nav2_msgs/action/assisted_teleop.hpp"


namespace nav2_recoveries
{
using AssistedTeleopAction = nav2_msgs::action::AssistedTeleop;

/**
 * @class nav2_recoveries::AssistedTeleop
 * @brief An action server recovery for assisted teleoperation for a fixed duration
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

  void onCleanup() override;

protected:
  std::chrono::time_point<std::chrono::steady_clock> assisted_teleop_end_;
  AssistedTeleopAction::Feedback::SharedPtr feedback_;

  /**
   * @brief Utility function to obtain robot pose
   * @return bool indicating availability of pose
   */
  bool updatePose();

  /**
   * @brief Utility function to project robot pose
   * @param speed_x linear speed in X
   * @param speed_y linear speed in y
   * @param speed_x angular speed about Z
   */
  void projectPose(double speed_x, double speed_y, double angular_vel_, double projection_time);

  /**
   * @brief Callback function for velocity subscriber
   * @param msg received Twist message
   */
  void vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  /**
   * @brief Check if pose is collision free
   */
  bool checkCollision();

  /**
   * @brief Move robot by specified velocity
   */
  void moveRobot();

  double speed_x = 0.0;
  double speed_y = 0.0;
  double angular_vel_ = 0;
  double projection_time = 1.0;
  double num_samples_ = 10;
  int loopcount = 1;

  geometry_msgs::msg::PoseStamped current_pose;
  geometry_msgs::msg::Pose2D projected_pose;

  double scaling_factor = 1;
  double col_time;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
  geometry_msgs::msg::Twist::UniquePtr cmd_vel_ = std::make_unique<geometry_msgs::msg::Twist>();
  std::unique_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_ros_;
};

}  // namespace nav2_recoveries

#endif  // NAV2_RECOVERIES__PLUGINS__ASSISTED_TELEOP_HPP_"
