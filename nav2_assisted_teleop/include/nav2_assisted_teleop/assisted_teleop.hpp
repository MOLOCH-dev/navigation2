/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Author(s): Shrijit Singh <shrijitsingh99@gmail.com>
 *
 */

#ifndef NAV2_ASSISTED_TELEOP__NAV2_ASSISTED_TELEOP_HPP_
#define NAV2_ASSISTED_TELEOP__NAV2_ASSISTED_TELEOP_HPP_

#include <string>
#include <vector>
#include <memory>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_assisted_teleop
{
  class AssistedTeleopController : public nav2_core::Controller
  {
    public:

      AssistedTeleopController () = default;

      ~AssistedTeleopController() override = default;

      void configure(
      const rclcpp_lifecycle::LifecycleNode::SharedPtr & parent,
      std::string name, const std::shared_ptr<tf2_ros::Buffer> & tf,
      const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros) override;
      
      void cleanup() override;

      void activate() override;

      void deactivate() override;


    geometry_msgs::msg::TwistStamped computeVelocityCommands(
      const geometry_msgs::msg::PoseStamped & pose,
      const geometry_msgs::msg::Twist & velocity) override;


    void setPlan(const nav_msgs::msg::Path & path) override;

    protected :
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::string plugin_name_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    nav2_costmap_2d::Costmap2D * costmap_;
    rclcpp::Logger logger_ {rclcpp::get_logger("AssistedTeleopController")};
    nav_msgs::msg::Path global_plan_;


  };
}  // namespace nav2_assisted_teleop

#endif  // NAV2_ASSISTED_TELEOP__NAV2_ASSISTED_TELEOP_HPP_