// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdint.h>
#include <chrono>
#include "emergency_mppi/controller.hpp"
#include "emergency_mppi/tools/utils.hpp"

// #define BENCHMARK_TESTING

namespace emergency_mppi
{

void MPPIController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_ = parent;
  costmap_ros_ = costmap_ros;
  tf_buffer_ = tf;
  name_ = name;
  parameters_handler_ = std::make_unique<ParametersHandler>(parent);

  auto node = parent_.lock();
  // The logger is inherited from the nav2_core::Controller base class.
  // It needs to be initialized from the parent node.
  logger_ = node->get_logger();
  // Set the logger level to WARN. This will show WARN, ERROR, and FATAL messages.
  logger_.set_level(rclcpp::Logger::Level::Warn);

  // Get high-level controller parameters
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(visualize_, "visualize", false);

  // Configure composed objects
  optimizer_.initialize(parent_, name_, costmap_ros_, parameters_handler_.get());
  path_handler_.initialize(parent_, name_, costmap_ros_, tf_buffer_, parameters_handler_.get());
  trajectory_visualizer_.on_configure(
    parent_, name_,
    costmap_ros_->getGlobalFrameID(), parameters_handler_.get());

  RCLCPP_INFO(logger_, "Configured MPPI Controller: %s", name_.c_str());
}

void MPPIController::cleanup()
{
  optimizer_.shutdown();
  trajectory_visualizer_.on_cleanup();
  parameters_handler_.reset();
  RCLCPP_INFO(logger_, "Cleaned up MPPI Controller: %s", name_.c_str());
}

void MPPIController::activate()
{
  trajectory_visualizer_.on_activate();
  parameters_handler_->start();
  RCLCPP_INFO(logger_, "Activated MPPI Controller: %s", name_.c_str());
}

void MPPIController::deactivate()
{
  trajectory_visualizer_.on_deactivate();
  RCLCPP_INFO(logger_, "Deactivated MPPI Controller: %s", name_.c_str());
}

void MPPIController::reset()
{
  RCLCPP_WARN(
    logger_,
    "[MPPI] MPPIController::reset() called by Nav2 (ControllerServer). "
    "This clears emergency mode if active. Cause: new navigation goal or plan replanning.");
  optimizer_.reset(false /*Don't reset zone-based speed limits between requests*/);
}

geometry_msgs::msg::TwistStamped MPPIController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  nav2_core::GoalChecker * goal_checker)
{
#ifdef BENCHMARK_TESTING
  auto start = std::chrono::system_clock::now();
#endif

  std::lock_guard<std::mutex> param_lock(*parameters_handler_->getLock());
  geometry_msgs::msg::Pose goal = path_handler_.getTransformedGoal(robot_pose.header.stamp).pose;

  nav_msgs::msg::Path transformed_plan;
  try {
    transformed_plan = path_handler_.transformPath(robot_pose);
  } catch (const nav2_core::InvalidPath & e) {
    // If we are in emergency mode, the path is not used by emergency critics.
    // We can proceed with an empty path.
    if (optimizer_.getEmergencyMode()) {
      RCLCPP_WARN(
        logger_,
        "Path is invalid, but controller is in emergency mode. Proceeding with empty path. "
        "Exception: %s", e.what());
    } else {
    // STATE: NORMAL
    // This is a critical failure in normal operation. Re-throw the exception
    // so the ControllerServer can catch it and abort the navigation task.
    RCLCPP_ERROR(
      logger_,
      "Path is invalid. Escalating to ControllerServer to handle failure. "
      "Exception: %s", e.what());
    throw; // This re-throws the original exception.
    }
  }

  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(*(costmap->getMutex()));

  geometry_msgs::msg::TwistStamped cmd =
    optimizer_.evalControl(robot_pose, robot_speed, transformed_plan, goal, goal_checker);

#ifdef BENCHMARK_TESTING
  auto end = std::chrono::system_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  RCLCPP_INFO(logger_, "Control loop execution time: %ld [ms]", duration);
#endif

  if (visualize_) {
    visualize(std::move(transformed_plan), cmd.header.stamp);
  }

  return cmd;
}

void MPPIController::visualize(
  nav_msgs::msg::Path transformed_plan,
  const builtin_interfaces::msg::Time & cmd_stamp)
{
  trajectory_visualizer_.add(optimizer_.getGeneratedTrajectories(), "Candidate Trajectories");
  trajectory_visualizer_.add(optimizer_.getOptimizedTrajectory(), "Optimal Trajectory", cmd_stamp);
  trajectory_visualizer_.visualize(std::move(transformed_plan));
}

void MPPIController::setPlan(const nav_msgs::msg::Path & path)
{
  path_handler_.setPath(path);
}

void MPPIController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  optimizer_.setSpeedLimit(speed_limit, percentage);
}

}  // namespace emergency_mppi

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(emergency_mppi::MPPIController, nav2_core::Controller)
