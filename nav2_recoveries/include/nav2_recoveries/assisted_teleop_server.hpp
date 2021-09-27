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

#ifndef NAV2_RECOVERIES__ASSISTED_TELEOP_SERVER_HPP_
#define NAV2_RECOVERIES__ASSISTED_TELEOP_SERVER_HPP_

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

namespace nav2_recoveries
{

using namespace std::chrono_literals;  //NOLINT

/**
 * @class nav2_recoveries::Recovery
 * @brief An action server recovery base class implementing the action server and basic factory.
 */
template<typename ActionT>
class Recovery : public nav2_core::Recovery
{
public:
  using ActionServer = nav2_util::SimpleActionServer<ActionT, rclcpp_lifecycle::LifecycleNode>;

  /**
   * @brief A Recovery constructor
   */
  Recovery()
  : action_server_(nullptr),
    cycle_frequency_(10.0),
    enabled_(false)
  {
  }

  virtual ~Recovery()
  {
  }

  // an opportunity for derived classes to do something on configuration
  // if they chose
  virtual void onConfigure()
  {
  }

  // an opportunity for derived classes to do something on cleanup
  // if they chose
  virtual void onCleanup()
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
      std::bind(&Recovery::execute, this));

    collision_checker_ = collision_checker;

    onConfigure();
  }

  // Cleanup server on lifecycle transition
  void cleanup() override
  {
    action_server_.reset();
    vel_pub_.reset();
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

  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_ros_;

protected:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

  std::string recovery_name_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  std::shared_ptr<ActionServer> action_server_;
  std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker> collision_checker_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::chrono::time_point<std::chrono::steady_clock> assisted_teleop_end_;

  double cycle_frequency_;
  double enabled_;
  std::string global_frame_;
  std::string robot_base_frame_;
  double transform_tolerance_;
  bool go = false;
  bool move = true;
  double scaling_factor;
  double speed_x = 0.0, speed_y = 0.0, angular_vel_ = 0.0;

  // Clock
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  // Logger
  rclcpp::Logger logger_{rclcpp::get_logger("nav2_recoveries")};


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
    auto feedback_ = std::make_shared<typename ActionT::Feedback>();
    auto result = std::make_shared<typename ActionT::Result>();

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
        moveRobot(scaling_factor, speed_x, speed_y, angular_vel_);
      } else {
        go = false;
        action_server_->succeeded_current(result);
        RCLCPP_INFO(
          logger_,
          "%s completed successfully", recovery_name_.c_str());
        return;
      }
    }
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

  // Stop the robot with a scaled-down velocity
  void
  moveRobot(double & scaling_factor, double speed_x, double speed_y, double angular_vel_)
  {
    auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();

    cmd_vel->linear.x = speed_x / (scaling_factor);
    cmd_vel->linear.y = speed_y;
    cmd_vel->angular.z = angular_vel_ / (scaling_factor);

    if (std::fabs(scaling_factor) >= 1 && move == true) {
      vel_pub_->publish(std::move(cmd_vel));
    }
  }
};

}  // namespace nav2_recoveries

#endif  // NAV2_RECOVERIES__ASSISTED_TELEOP_SERVER_HPP_
