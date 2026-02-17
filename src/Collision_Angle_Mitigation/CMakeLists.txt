cmake_minimum_required(VERSION 3.8)
project(Collision_Angle_Mitigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_mppi_controller REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)

include_directories(include)

add_library(collision_angle_critic SHARED
  src/collision_angle_critic.cpp
)

target_include_directories(collision_angle_critic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(collision_angle_critic
  rclcpp
  nav2_mppi_controller
  pluginlib
  nav2_core
)

pluginlib_export_plugin_description_file(nav2_mppi_controller plugins.xml)

install(TARGETS collision_angle_critic
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch models params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()