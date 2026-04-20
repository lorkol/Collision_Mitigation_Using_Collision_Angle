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
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "angles/angles.h"
#include <omp.h>
#include <cstring>
#include <atomic>
#include <limits>

namespace emergency_mppi::critics
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

  unsigned int size_x = costmap->getSizeInCellsX();
  unsigned int size_y = costmap->getSizeInCellsY();
  size_t total_cells = size_x * size_y;

  if (grad_cache_flag_.size() != total_cells) {
    grad_cache_val_.resize(total_cells);
    grad_cache_yaw_.resize(total_cells);
    grad_cache_flag_.resize(total_cells);
  }
  std::memset(grad_cache_flag_.data(), 0, total_cells * sizeof(uint8_t));

  // Extract trajectory and state tensors for xtensor operations
  const size_t batch_size = data.trajectories.x.shape(0);
  const size_t time_steps = data.trajectories.x.shape(1);

  std::atomic<int> grad_hit_count{0};
  std::atomic<int> penalized_count{0};
  std::atomic<float> total_cost_added{0.0f};
  std::atomic<float> min_traj_cost{std::numeric_limits<float>::max()};
  std::atomic<float> max_traj_cost{0.0f};
  std::atomic<int> t0_breaks{0};  // how many trajectories break at t=0

  #pragma omp parallel for
  for (int i = 0; i < static_cast<int>(batch_size); ++i) {
    float trajectory_cost = 0.0f;
    geometry_msgs::msg::Pose robot_pose;
    geometry_msgs::msg::PoseStamped gradient_pose;
    tf2::Quaternion q;
    // Iterate through time steps
    for (size_t t = 0; t < time_steps; ++t) {
      robot_pose.position.x = data.trajectories.x(i, t);
      robot_pose.position.y = data.trajectories.y(i, t);
      double robot_yaw = data.trajectories.yaws(i, t);

      q.setRPY(0, 0, robot_yaw);
      robot_pose.orientation = tf2::toMsg(q);

      unsigned int mx, my;
      if (costmap->worldToMap(robot_pose.position.x, robot_pose.position.y, mx, my)) {
        unsigned int index = my * size_x + mx;
        
        // Atomic load with acquire to ensure we see data written before the flag
        uint8_t flag = __atomic_load_n(&grad_cache_flag_[index], __ATOMIC_ACQUIRE);
        
        float val;
        double grad_yaw;
        bool has_grad = false;

        if (flag == 1) {
          val = grad_cache_val_[index];
          grad_yaw = grad_cache_yaw_[index];
          has_grad = true;
        } else if (flag == 0) {
          // Not computed yet
          MapIndex idx;
          if (edt_estimator_->getMinEDT(robot_pose, edt_map, costmap, idx, val)) {
            if (val < 1.0f) {
              if (edt_estimator_->getGrad(edt_map, costmap, idx, gradient_pose)) {
                grad_yaw = tf2::getYaw(gradient_pose.pose.orientation);
                
                grad_cache_val_[index] = val;
                grad_cache_yaw_[index] = static_cast<float>(grad_yaw);
                // Atomic store with release to ensure data is visible before flag is set
                __atomic_store_n(&grad_cache_flag_[index], 1, __ATOMIC_RELEASE);
                has_grad = true;
              }
            }
          }
          if (!has_grad) {
            // Mark as invalid/no-grad so other threads don't recompute
            __atomic_store_n(&grad_cache_flag_[index], 2, __ATOMIC_RELEASE);
          }
        }
        // If flag == 2, we skip (has_grad remains false)

        if (has_grad) {
            grad_hit_count.fetch_add(1, std::memory_order_relaxed);

            // Calculate velocity vector from state (Robot Frame)
            double vx = data.state.vx(i, t);
            double vy = data.state.vy(i, t);

            double vel_yaw;
            // If velocity is negligible, fallback to robot heading
            if (std::hypot(vx, vy) < 1e-3) {
              vel_yaw = robot_yaw;
            } else {
              // Convert robot-frame velocity to global-frame heading
              // This is equivalent to rotating the vector (vx, vy) by robot_yaw
              vel_yaw = robot_yaw + std::atan2(vy, vx);
            }

            double diff = angles::shortest_angular_distance(vel_yaw, grad_yaw);

            // Gradient points AWAY from obstacle (Ascent).
            // We want to penalize moving TOWARDS obstacle (Descent).
            // Alignment = 1.0 (Moving Away), -1.0 (Moving Towards)
            double alignment = std::cos(diff);
            double dot_product = std::sqrt(std::pow(vx, 2) + std::pow(vy, 2)) * alignment;

            // TODO : Change the actual cost function to work with the nav2 costs given according to collision etc
            if (alignment < 0.0) {
              trajectory_cost += weight_ * std::pow(-dot_product, power_) * data.model_dt;
              penalized_count.fetch_add(1, std::memory_order_relaxed);
            }
            if (t == 0) { t0_breaks.fetch_add(1, std::memory_order_relaxed); }
            t = time_steps; // Break inner loop if we are within the threshold to save computation on this trajectory.
        }
      }
    }
    data.costs[i] += trajectory_cost;
    float prev = total_cost_added.load(std::memory_order_relaxed);
    while (!total_cost_added.compare_exchange_weak(prev, prev + trajectory_cost,
      std::memory_order_relaxed)) {}
    float mn = min_traj_cost.load(std::memory_order_relaxed);
    while (trajectory_cost < mn &&
      !min_traj_cost.compare_exchange_weak(mn, trajectory_cost, std::memory_order_relaxed)) {}
    float mx = max_traj_cost.load(std::memory_order_relaxed);
    while (trajectory_cost > mx &&
      !max_traj_cost.compare_exchange_weak(mx, trajectory_cost, std::memory_order_relaxed)) {}
  }

  fprintf(stderr,
    "[CollisionAngleCritic] grad_hits=%d penalized=%d t0_breaks=%d traj_cost min=%.4f max=%.4f total=%.2f\n",
    grad_hit_count.load(), penalized_count.load(), t0_breaks.load(),
    min_traj_cost.load(), max_traj_cost.load(), total_cost_added.load());
  fflush(stderr);

}

}  // namespace emergency_mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(emergency_mppi::critics::CollisionAngleCritic, emergency_mppi::critics::CriticFunction)
