cmake_minimum_required(VERSION 3.8)
project(Collision_Angle_Mitigation)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_mppi_controller REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(OpenCV REQUIRED)

# Build the critic plugin as a shared library
add_library(collision_angle_critic SHARED
  src/collision_angle_critic.cpp)
target_include_directories(collision_angle_critic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(collision_angle_critic
  rclcpp
  nav2_mppi_controller
  nav2_core
  pluginlib)

# Build the EDT publisher as an executable
add_executable(edt_publisher_node src/edt_publisher_node.cpp)
ament_target_dependencies(edt_publisher_node
  rclcpp
  nav_msgs
  rcl_interfaces
  nav2_costmap_2d
  OpenCV)

# Install library and executable
install(TARGETS
  collision_angle_critic
  DESTINATION lib)
install(TARGETS
  edt_publisher_node
  DESTINATION lib/${PROJECT_NAME})

# Install headers
install(
  DIRECTORY include/
  DESTINATION include)

# Install and export the plugin definition
install(
  FILES plugins.xml
  DESTINATION share/${PROJECT_NAME})
pluginlib_export_plugin_description_file(nav2_mppi_controller plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()