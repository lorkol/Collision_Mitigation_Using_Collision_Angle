cmake_minimum_required(VERSION 3.8)
project(collision_angle_optimizer)

# 1. Standard C++ 17 and compiler flags for ROS 2 Jazzy
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# 2. Find necessary dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_mppi REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_costmap_2d REQUIRED)

# 3. Create the shared library for your Critic plugin
add_library(collision_angle_critic_lib SHARED
  src/collision_angle_critic.cpp
)

# 4. Set up include directories
target_include_directories(collision_angle_critic_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 5. Link the library to Nav2 and ROS 2 Jazzy headers
ament_target_dependencies(collision_angle_critic_lib
  rclcpp
  nav2_core
  nav2_mppi
  pluginlib
  nav2_costmap_2d
)

# 6. Install the compiled C++ library
install(TARGETS collision_angle_critic_lib
  DESTINATION lib/
)

# 7. Install the XML description file (Required for pluginlib)
install(FILES mppi_critics.xml
  DESTINATION share/${PROJECT_NAME}/
)

# 8. Install your custom Launch and Parameter folders (The .py and .yaml files)
install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
