cmake_minimum_required(VERSION 3.8)
project(collision_angle_mitigation)

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
find_package(emergency_mppi REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(OpenMP REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FloatEDM.msg"
  DEPENDENCIES std_msgs nav_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Build the EDT Layer plugin
add_library(edt_layer SHARED src/edt_layer.cpp)
target_include_directories(edt_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(edt_layer
  rclcpp
  nav2_costmap_2d
  rclcpp_lifecycle
  pluginlib
  OpenCV)
target_link_libraries(edt_layer "${cpp_typesupport_target}")

# Build Collision Mitigation Utils library (Gradient Estimator + Footprint)
add_library(collision_mitigation_utils SHARED
  src/edt_gradient_estimator.cpp
  src/robot_footprint.cpp)
target_include_directories(collision_mitigation_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(collision_mitigation_utils
  rclcpp
  nav2_costmap_2d
  geometry_msgs
  nav_msgs
  tf2_geometry_msgs
  OpenCV)
target_link_libraries(collision_mitigation_utils edt_layer) # Links against EdtLayer to use its class definition

# Build the EDT publisher as an executable
add_executable(edt_publisher_node src/edt_publisher_node.cpp)
ament_target_dependencies(edt_publisher_node
  rclcpp
  nav_msgs
  rcl_interfaces
  nav2_costmap_2d
  OpenCV)
target_link_libraries(edt_publisher_node "${cpp_typesupport_target}")

# Build the EDT monitor node
add_executable(edt_monitor_node
  src/edt_monitor_node.cpp # Note: robot_footprint.cpp removed as it's now in edt_utils
)
target_include_directories(edt_monitor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(edt_monitor_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs)
target_link_libraries(edt_monitor_node "${cpp_typesupport_target}" collision_mitigation_utils)


# Build the critic plugin as a shared library
add_library(collision_angle_critic SHARED
  src/collision_angle_critic.cpp)
target_include_directories(collision_angle_critic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(collision_angle_critic
  rclcpp
  emergency_mppi
  nav2_core
  pluginlib
  nav2_costmap_2d
  angles
  tf2
  tf2_geometry_msgs)
if(OpenMP_CXX_FOUND)
  target_link_libraries(collision_angle_critic OpenMP::OpenMP_CXX)
endif()
target_link_libraries(collision_angle_critic collision_mitigation_utils emergency_mppi::emergency_mppi_lib)

# Install library and executable
install(TARGETS
  collision_angle_critic
  edt_layer
  collision_mitigation_utils
  DESTINATION lib)
install(TARGETS
  edt_publisher_node
  edt_monitor_node
  DESTINATION lib/${PROJECT_NAME})

# Install headers
install(
  DIRECTORY include/
  DESTINATION include)

install(
  DIRECTORY behavior_trees /
  DESTINATION share/${PROJECT_NAME})

# Install and export the plugin definition
install(
  FILES plugins.xml costmap_plugins.xml
  DESTINATION share/${PROJECT_NAME})
pluginlib_export_plugin_description_file(emergency_mppi plugins.xml)
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch models params urdf worlds configs maps rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()