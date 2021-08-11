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

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <queue>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <streambuf>
#include <algorithm>
#include <limits>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2_ros/transform_listener.h"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/time.hpp"
#include "tf2/convert.h"
#include "tf2_ros/buffer.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"

#include "nav2_util/robot_utils.hpp"
#include "nav2_util/string_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_assisted_teleop/assisted_teleop.hpp"

using namespace std::chrono_literals;

namespace nav2_assisted_teleop
{

AssistedTeleop::AssistedTeleop()
: nav2_util::LifecycleNode("AssistedTeleop", "", false)
{
  logger_ = get_logger();
  RCLCPP_INFO(logger_, "Creating");
  declare_parameter("lookahead_time", 1);
  declare_parameter("global_frame", "map");
}

AssistedTeleop::~AssistedTeleop()
{
}

nav2_util::CallbackReturn
AssistedTeleop::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Configuring");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
AssistedTeleop::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Activating");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
AssistedTeleop::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Deactivating");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
AssistedTeleop::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(logger_, "Cleaning up");

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
AssistedTeleop::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(logger_, "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

// Get Parameters

void
AssistedTeleop::timer_callback()
{
}
}  // namespace nav2_assisted_teleop
