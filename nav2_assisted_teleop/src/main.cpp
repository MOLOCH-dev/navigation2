#include <memory>
#include <string>
#include "nav2_assisted_teleop/assisted_teleop.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto assisted_teleop_node = std::make_shared<nav2_assisted_teleop::AssistedTeleop>();
  rclcpp::spin(assisted_teleop_node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}