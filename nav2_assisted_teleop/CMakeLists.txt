cmake_minimum_required(VERSION 3.5)
project(nav2_assisted_teleop)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

nav2_package()

include_directories(
  include
)

set(dependencies
  rclcpp
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  nav_msgs
  nav2_util
  nav2_core
  tf2
)

add_library(nav2_assisted_teleop_core SHARED
        src/assisted_teleop.cpp)

add_executable(nav2_assisted_teleop src/main.cpp)

# prevent pluginlib from using boost
# target_compile_definitions(nav2_assisted_teleop PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(nav2_assisted_teleop_core
  ${dependencies}
)


ament_target_dependencies(nav2_assisted_teleop
  ${dependencies}
)

target_link_libraries(nav2_assisted_teleop
  nav2_assisted_teleop_core
)



install(TARGETS nav2_assisted_teleop_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
nav2_assisted_teleop 
RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(nav2_assisted_teleop_core)
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core nav2_assisted_teleop.xml)

ament_package()

