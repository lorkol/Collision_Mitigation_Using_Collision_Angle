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

#include "collision_angle_critic.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "edt_gradient_estimator.hpp"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "angles/angles.h"

namespace mppi::critics
{

void CollisionAngleCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  auto getParentParam = parameters_handler_->getParamGetter(parent_name_);

  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 0.0f);

  float radius;
  // TODO: Get Robot Radius from parameters
  getParam(radius, "robot_radius", 0.17f);

  edt_estimator_ = std::make_unique<edt_gradient_estimator::EdtGradientEstimator>(radius);

  RCLCPP_INFO(
    logger_, "CollisionAngleCritic instantiated with %d power and %f weight.",
    power_, weight_);

  float vx_max, vy_max, vx_min;
  getParentParam(vx_max, "vx_max", 0.5f);
  getParentParam(vy_max, "vy_max", 0.0f);
  getParentParam(vx_min, "vx_min", -0.35f);

  const float min_sgn = vx_min > 0.0f ? 1.0f : -1.0f;
  max_vel_ = sqrtf(vx_max * vx_max + vy_max * vy_max);
  min_vel_ = min_sgn * sqrtf(vx_min * vx_min + vy_max * vy_max);

  auto * layered_costmap = costmap_ros_->getLayeredCostmap();
  auto plugins = layered_costmap->getPlugins();
  for (auto plugin : *plugins) {
    auto edt_layer = std::dynamic_pointer_cast<collision_angle_mitigation::EdtLayer>(plugin);
    if (edt_layer) {
      edt_layer_ = edt_layer.get();
      break;
    }
  }

  if (!edt_layer_) {
    RCLCPP_WARN(logger_, "CollisionAngleCritic: EdtLayer not found in costmap. Ensure it is added to the plugins.");
  }
}

void CollisionAngleCritic::score(CriticData & data)
{
  if (!enabled_ || !edt_layer_) {
    return;
  }

  // Get the latest EDT map (Thread-safe access)
  cv::Mat edt_map = edt_layer_->getEdt();
  if (edt_map.empty()) {
    return;
  }

  // Get Map Metadata
  auto * costmap = edt_layer_->getCostmap();


  // Extract trajectory and state tensors for xtensor operations
  const size_t batch_size = data.trajectories.x.shape(0);
  const size_t time_steps = data.trajectories.x.shape(1);

  geometry_msgs::msg::Pose robot_pose;
  geometry_msgs::msg::PoseStamped gradient_pose;
  tf2::Quaternion q;

  for (size_t i = 0; i < batch_size; ++i) {
    float trajectory_cost = 0.0f;
    // Iterate up to time_steps - 1 to calculate velocity from position differences
    for (size_t t = 0; t < time_steps - 1; ++t) {
      robot_pose.position.x = data.trajectories.x(i, t);
      robot_pose.position.y = data.trajectories.y(i, t);
      double robot_yaw = data.trajectories.yaws(i, t);

      q.setRPY(0, 0, robot_yaw);
      robot_pose.orientation = tf2::toMsg(q);

      MapIndex idx;
      float val;
      if (edt_estimator_->getMinEDT(robot_pose, edt_map, costmap, idx, val)) {
        if (val < 1.0f || latency_testing_) {
          if (edt_estimator_->getGrad(edt_map, costmap, idx, gradient_pose)) {
            double grad_yaw = tf2::getYaw(gradient_pose.pose.orientation);

            // Calculate velocity vector from trajectory points
            float dx = data.trajectories.x(i, t + 1) - data.trajectories.x(i, t);
            float dy = data.trajectories.y(i, t + 1) - data.trajectories.y(i, t);

            double vel_yaw;
            // If displacement is negligible, fallback to robot heading
            if (std::hypot(dx, dy) < 1e-3) {
              vel_yaw = robot_yaw;
            } else {
              vel_yaw = std::atan2(dy, dx);
            }

            double diff = angles::shortest_angular_distance(vel_yaw, grad_yaw);

            // Gradient points AWAY from obstacle (Ascent).
            // We want to penalize moving TOWARDS obstacle (Descent).
            // Alignment = 1.0 (Moving Away), -1.0 (Moving Towards)
            double alignment = std::cos(diff);

            // TODO remove the false condition to enable penalization of moving towards obstacles. Currently left off for testing purposes.
            if (alignment < 0.0 && false) {
              trajectory_cost += weight_ * std::pow(-alignment, power_) * data.model_dt;
            }
            if (!latency_testing_) t = time_steps; // Break inner loop if we are within the threshold to save computation on this trajectory.
          }
        }
      }
    }
    data.costs[i] += trajectory_cost;
  }
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::CollisionAngleCritic, mppi::critics::CriticFunction)
