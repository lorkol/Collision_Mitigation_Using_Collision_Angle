cmake_minimum_required(VERSION 3.8)
project(emergency_mppi)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_common REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(angles REQUIRED)
find_package(xtensor REQUIRED)
find_package(xsimd REQUIRED)
find_package(OpenMP)

# If nav2_common macros are available, use them, otherwise standard cmake
include_directories(
  include
  ${xtensor_INCLUDE_DIRS}
  ${xsimd_INCLUDE_DIRS}
)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  nav2_msgs
  nav2_util
  nav2_common
  geometry_msgs
  visualization_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  pluginlib
  angles
)

# Find all source files recursively (handles critics/ subfolders)
file(GLOB_RECURSE SOURCES src/*.cpp)

add_library(emergency_mppi_lib SHARED ${SOURCES})
target_include_directories(emergency_mppi_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(emergency_mppi_lib ${dependencies})

# Link OpenMP and Xtensor
if(OpenMP_FOUND)
  target_link_libraries(emergency_mppi_lib OpenMP::OpenMP_CXX)
endif()
target_link_libraries(emergency_mppi_lib xtensor xsimd)
# Optimization flags often used with MPPI/Xtensor
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(emergency_mppi_lib PRIVATE -march=native -mavx2)
endif()

# Register the plugin file so ROS knows about it
pluginlib_export_plugin_description_file(nav2_core mppi_plugin.xml)
pluginlib_export_plugin_description_file(emergency_mppi mppi_plugin.xml)

# Install rules
install(TARGETS emergency_mppi_lib
  EXPORT emergency_mppi_lib_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES mppi_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(EXPORT emergency_mppi_lib_targets
  FILE emergency_mppi_lib_targets.cmake
  NAMESPACE emergency_mppi::
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_targets(emergency_mppi_lib_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies} xtensor OpenMP)

ament_package()
