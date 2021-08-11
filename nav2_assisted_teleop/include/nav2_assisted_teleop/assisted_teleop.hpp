/*
 *  Copyright (c) 2008.
 *  All rights reserved.
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/* Author: Anushree Sabnis */

#ifndef NAV2_ASSISTED_TELEOP__ASSISTED_TELEOP_HPP_
#define NAV2_ASSISTED_TELEOP__ASSISTED_TELEOP_HPP_

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_util/robot_utils.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/node_thread.hpp"
#include "nav2_util/lifecycle_node.hpp"


namespace nav2_assisted_teleop
{
class AssistedTeleop : public nav2_util::LifecycleNode
{
public:
    /**
     * @brief A constructor for nav2_assisted_teleop::AssistedTeleop class
     */
    AssistedTeleop();
    /**
     * @brief A destructor fornav2_assisted_teleop::AssistedTeleop class
     */
    ~AssistedTeleop();

    /**
     * @brief Configures member variables
     *
     * Initializes action server for "follow_waypoints"
     * @param state Reference to LifeCycle node state
     * @return SUCCESS or FAILURE
     */
    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    /**
     * @brief Activates action server
     * @param state Reference to LifeCycle node state
     * @return SUCCESS or FAILURE
     */
    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    /**
     * @brief Deactivates action server
     * @param state Reference to LifeCycle node state
     * @return SUCCESS or FAILURE
     */
    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    /**
     * @brief Resets member variables
     * @param state Reference to LifeCycle node state
     * @return SUCCESS or FAILURE
     */
    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
    /**
     * @brief Called when in shutdown state
     * @param state Reference to LifeCycle node state
     * @return SUCCESS or FAILURE
     */
    nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

    /**
       * @brief Get the pose of the robot in the global frame of the costmap
       * @param global_pose Will be set to the pose of the robot in the global frame of the costmap
       * @return True if the pose was set successfully, false otherwise
       */
    // bool getRobotPose(geometry_msgs::msg::PoseStamped & global_pose);
    // /**
    // The Logger object for logging
    rclcpp::Logger logger_{rclcpp::get_logger("nav2_assisted_teleop")};

protected:
    // The local node
    rclcpp::Node::SharedPtr rclcpp_node_;
    /**
     * @brief Get parameters for node
     */



    /**
     * @brief Initialize required ROS transformations
     */
    void initTransforms();
    void timer_callback();
    void subscriptionListenerThreadLoop();
};
}  // end namespace nav2_assisted_teleop
#endif  // NAV2_ASSISTED_TELEOP__ASSISTED_TELEOP_HPP_
