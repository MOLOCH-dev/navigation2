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

#include <memory>
#include <string>
#include <cmath>
#include <chrono>
#include <ctime>
#include <thread>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_core/recovery.hpp"

#include "nav2_msgs/action/assisted_teleop.hpp"

namespace nav2_recoveries
{

using namespace std::chrono_literals;  //NOLINT

/**
 * @class nav2_recoveries::AssistedTeleop
 * @brief An action server recovery base class implementing the action server and basic factory.
 */
class AssistedTeleop : public nav2_core::Recovery
{
public:
  using AssistedTeleopAction = nav2_msgs::action::AssistedTeleop;
  using ActionServer = nav2_util::SimpleActionServer<AssistedTeleopAction,
      rclcpp_lifecycle::LifecycleNode>;

  /**
   * @brief AsistedTeleop constructor
   */
  AssistedTeleop()
  : action_server_(nullptr),
    cycle_frequency_(10.0),
    enabled_(false)
  {
  }

  ~AssistedTeleop()
  {
  }

  // Configuration of Assisted Teleop Action
  void onConfigure();

  // Cleaning up subscribers
  void onCleanup()
  {
  }

  // configure the server on lifecycle setup
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker) override
  {
    node_ = parent;
    auto node = node_.lock();

    logger_ = node->get_logger();

    RCLCPP_INFO(logger_, "Configuring %s", name.c_str());

    recovery_name_ = name;
    tf_ = tf;

    node->get_parameter("cycle_frequency", cycle_frequency_);
    node->get_parameter("global_frame", global_frame_);
    node->get_parameter("robot_base_frame", robot_base_frame_);
    node->get_parameter("transform_tolerance", transform_tolerance_);

    action_server_ = std::make_shared<ActionServer>(
      node, recovery_name_,
      std::bind(&AssistedTeleop::execute, this));

    collision_checker_ = collision_checker;

    onConfigure();
  }

  // Cleanup server on lifecycle transition
  void cleanup() override
  {
    action_server_.reset();
    vel_pub_.reset();
    vel_sub_.reset();
    onCleanup();
  }

  // Activate server on lifecycle transition
  void activate() override
  {
    RCLCPP_INFO(logger_, "Activating %s", recovery_name_.c_str());

    vel_pub_->on_activate();
    action_server_->activate();
    enabled_ = true;
  }

  // Deactivate server on lifecycle transition
  void deactivate() override
  {
    vel_pub_->on_deactivate();
    action_server_->deactivate();
    enabled_ = false;
  }

protected:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

  // Action Server
  std::shared_ptr<ActionServer> action_server_;

  // Publishers and Subscribers
  std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
  std::unique_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;

  // User defined parameters
  double projection_time_;
  double linear_velocity_threshold_;
  double cycle_frequency_;
  double enabled_;
  double transform_tolerance_;
  std::string global_frame_;
  std::string robot_base_frame_;
  std::string cmd_vel_topic_;

  geometry_msgs::msg::PoseStamped current_pose;
  geometry_msgs::msg::Pose2D projected_pose;
  std::string recovery_name_;
  std::chrono::time_point<std::chrono::steady_clock> assisted_teleop_end_;
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_ros_;

  // Parameters for Assisted Teleop
  bool go = false;
  double scaling_factor = 1;
  double speed_x = 0.0, speed_y = 0.0, angular_vel_ = 0.0;

  // Clock
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  // Logger
  rclcpp::Logger logger_{rclcpp::get_logger("nav2_recoveries")};

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

  // Main execution callbacks for the action server implementation waiting for timeout
  // and enabling the recovery's specific behaviour in the meantime
  void execute()
  {
    RCLCPP_INFO(logger_, "Attempting %s", recovery_name_.c_str());

    if (!enabled_) {
      RCLCPP_WARN(
        logger_,
        "Called while inactive, ignoring request.");
      return;
    }

    // Log a message every second
    {
      auto node = node_.lock();
      if (!node) {
        throw std::runtime_error{"Failed to lock node"};
      }

      auto timer = node->create_wall_timer(
        1s,
        [&]()
        {RCLCPP_INFO(logger_, "%s running...", recovery_name_.c_str());});
    }

    auto start_time = steady_clock_.now();

    // Initialize the ActionT goal, feedback and result
    auto at_goal = action_server_->get_current_goal();
    auto feedback_ = std::make_shared<AssistedTeleopAction::Feedback>();
    auto result = std::make_shared<AssistedTeleopAction::Result>();

    rclcpp::WallRate loop_rate(cycle_frequency_);

    assisted_teleop_end_ = std::chrono::steady_clock::now() +
      rclcpp::Duration(at_goal->time).to_chrono<std::chrono::nanoseconds>();

    while (rclcpp::ok()) {
      if (action_server_->is_cancel_requested()) {
        RCLCPP_INFO(logger_, "Canceling %s", recovery_name_.c_str());
        go = false;
        stopRobot();
        result->total_elapsed_time = steady_clock_.now() - start_time;
        action_server_->terminate_all(result);
        return;
      }

      if (action_server_->is_preempt_requested()) {
        RCLCPP_ERROR(
          logger_, "Received a preemption request for %s,"
          " however feature is currently not implemented. Aborting and stopping.",
          recovery_name_.c_str());
        stopRobot();
        result->total_elapsed_time = steady_clock_.now() - start_time;
        action_server_->terminate_current(result);
        return;
      }

      auto current_point = std::chrono::steady_clock::now();

      auto time_left =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
        assisted_teleop_end_ - current_point).count();

      feedback_->time_left = rclcpp::Duration(
        rclcpp::Duration::from_nanoseconds(time_left));

      action_server_->publish_feedback(feedback_);

      // Enable recovery behavior if we haven't run out of time
      if (time_left > 0) {
        go = true;
        costmap_ros_ = costmap_sub_->getCostmap();
      } else {
        go = false;
        action_server_->succeeded_current(result);
        RCLCPP_INFO(
          logger_,
          "%s completed successfully", recovery_name_.c_str());
        return;
      }
    }

    // loop_rate.sleep();
  }

  // Stop the robot with a commanded velocity
  void stopRobot()
  {
    auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
    cmd_vel->linear.x = 0.0;
    cmd_vel->linear.y = 0.0;
    cmd_vel->angular.z = 0.0;

    vel_pub_->publish(std::move(cmd_vel));
  }
};

}  // namespace nav2_recoveries

#endif  // NAV2_RECOVERIES__PLUGINS__ASSISTED_TELEOP_HPP_
