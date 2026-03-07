#include "edt_gradient_estimator.hpp"
#include <limits>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace collision_angle_mitigation;
namespace edt_gradient_estimator
{

// Helper to safely access map data with bounds checking
double getMapValue(int ix, int iy, const cv::Mat & edt_map){
    // Check if coordinates are outside map boundaries
    if (ix < 0 || ix >= edt_map.cols || iy < 0 || iy >= edt_map.rows) return -1.0;
    // Return the value at the calculated linear index    
    return static_cast<double>(edt_map.at<float>(iy, ix));
}

EdtGradientEstimator::EdtGradientEstimator(double robot_radius)
{
  footprint_model_ = std::make_unique<CircularFootprint>(robot_radius);
}

EdtGradientEstimator::~EdtGradientEstimator() = default;

bool EdtGradientEstimator::getMinEDT(
  const geometry_msgs::msg::Pose & robot_pose,
  const cv::Mat & edt_map,
  const nav_msgs::msg::MapMetaData & map_info,
  MapIndex & out_min_idx,
  float & out_min_val)
{
  if (edt_map.empty()) {
    return false;
  }

  // Get indices under footprint
  std::vector<MapIndex> footprint_indices;
  footprint_model_->getIndices(robot_pose, map_info, footprint_indices);
  if (footprint_indices.empty()) {
    return false;
  }

  // Find point with Minimum EDT value
  float min_val = std::numeric_limits<float>::max();
  MapIndex min_idx = {-1, -1};

  for (const auto & idx : footprint_indices) {
    // Check bounds (getIndices usually checks, but safety first)
    if (idx.x >= 0 && idx.x < static_cast<int>(map_info.width) &&
        idx.y >= 0 && idx.y < static_cast<int>(map_info.height))
    {
      float val = edt_map.at<float>(idx.y, idx.x);
      // Assuming < 0 is invalid/unknown if that convention persists, 
      // though distanceTransform usually returns >= 0.
      if (val < min_val) {
        min_val = val;
        min_idx = idx;
      }
    }
  }

  if (min_idx.x == -1) {
    return false;
  }

  out_min_idx = min_idx;
  out_min_val = min_val;
  return true;
}

bool EdtGradientEstimator::getMinEDT(
  const geometry_msgs::msg::Pose & robot_pose,
  const cv::Mat & edt_map,
  const nav2_costmap_2d::Costmap2D * costmap,
  MapIndex & out_min_idx,
  float & out_min_val)
{
  if (edt_map.empty()) {
    return false;
  }

  // Get indices under footprint
  std::vector<MapIndex> footprint_indices;
  footprint_model_->getIndices(robot_pose, *costmap, footprint_indices);
  if (footprint_indices.empty()) {
    return false;
  }

  // Find point with Minimum EDT value
  float min_val = std::numeric_limits<float>::max();
  MapIndex min_idx = {-1, -1};

  for (const auto & idx : footprint_indices) {
    // Check bounds (getIndices usually checks, but safety first)
    if (idx.x >= 0 && idx.x < static_cast<int>(costmap->getSizeInCellsX()) &&
        idx.y >= 0 && idx.y < static_cast<int>(costmap->getSizeInCellsY()))
    {
      float val = edt_map.at<float>(idx.y, idx.x);
      if (val < min_val) {
        min_val = val;
        min_idx = idx;
      }
    }
  }

  if (min_idx.x == -1) {
    return false;
  }

  out_min_idx = min_idx;
  out_min_val = min_val;
  return true;
}

bool EdtGradientEstimator::getGrad(
  const cv::Mat & edt_map,
  const nav_msgs::msg::MapMetaData & map_info,
  const MapIndex & min_idx,
  geometry_msgs::msg::PoseStamped & out_gradient_pose)
{
  if (edt_map.empty()) {
    return false;
  }
  
  // Calculate Gradient
  double center_val = getMapValue(min_idx.x, min_idx.y, edt_map);
  int best_dx = 0;
  int best_dy = 0;
  double max_slope = 0.0;

  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      if (dx == 0 && dy == 0) continue;
      
      int nx = min_idx.x + dx;
      int ny = min_idx.y + dy;

      if (nx >= 0 && nx < static_cast<int>(map_info.width) &&
          ny >= 0 && ny < static_cast<int>(map_info.height))
      {
        double val = getMapValue(nx, ny, edt_map);
        if (val < 0.0f) continue;
        double diff = center_val - val; // Positive if neighbor is smaller (descent)
        if (diff > 0) {
          double dist = (dx != 0 && dy != 0) ? 1.41421356 : 1.0;
          double slope = diff / dist;
          if (slope > max_slope) {
            max_slope = slope;
            best_dx = dx;
            best_dy = dy;
          }
        }
      }
    }
  }

  // Populate Result
  out_gradient_pose.header.frame_id = "odom";
  out_gradient_pose.header.stamp = rclcpp::Time(0); // TODO: Caller should set time

  out_gradient_pose.pose.position.x = map_info.origin.position.x + (min_idx.x + 0.5) * map_info.resolution;
  out_gradient_pose.pose.position.y = map_info.origin.position.y + (min_idx.y + 0.5) * map_info.resolution;
  out_gradient_pose.pose.position.z = 0.0;

  double yaw = std::atan2(static_cast<double>(best_dy), static_cast<double>(best_dx));
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw + M_PI); // Pointing towards steepest descent
  out_gradient_pose.pose.orientation = tf2::toMsg(q);

  return true;
}

bool EdtGradientEstimator::getGrad(
  const cv::Mat & edt_map,
  const nav2_costmap_2d::Costmap2D * costmap,
  const MapIndex & min_idx,
  geometry_msgs::msg::PoseStamped & out_gradient_pose)
{
  if (edt_map.empty()) {
    return false;
  }
  
  // Calculate Gradient
  double center_val = getMapValue(min_idx.x, min_idx.y, edt_map);
  int best_dx = 0;
  int best_dy = 0;
  double max_slope = 0.0;

  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      if (dx == 0 && dy == 0) continue;
      
      int nx = min_idx.x + dx;
      int ny = min_idx.y + dy;

      if (nx >= 0 && nx < static_cast<int>(costmap->getSizeInCellsX()) &&
          ny >= 0 && ny < static_cast<int>(costmap->getSizeInCellsY()))
      {
        double val = getMapValue(nx, ny, edt_map);
        if (val < 0.0f) continue;
        double diff = center_val - val; // Positive if neighbor is smaller (descent)
        if (diff > 0) {
          double dist = (dx != 0 && dy != 0) ? 1.41421356 : 1.0;
          double slope = diff / dist;
          if (slope > max_slope) {
            max_slope = slope;
            best_dx = dx;
            best_dy = dy;
          }
        }
      }
    }
  }

  // Populate Result
  out_gradient_pose.header.frame_id = "odom"; // TODO: Should match costmap frame
  out_gradient_pose.header.stamp = rclcpp::Time(0);

  out_gradient_pose.pose.position.x = costmap->getOriginX() + (min_idx.x + 0.5) * costmap->getResolution();
  out_gradient_pose.pose.position.y = costmap->getOriginY() + (min_idx.y + 0.5) * costmap->getResolution();
  out_gradient_pose.pose.position.z = 0.0;

  double yaw = std::atan2(static_cast<double>(best_dy), static_cast<double>(best_dx));
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw + M_PI); // Pointing towards steepest descent
  out_gradient_pose.pose.orientation = tf2::toMsg(q);

  return true;
}

bool EdtGradientEstimator::calculate(
  const geometry_msgs::msg::Pose & robot_pose,
  const cv::Mat & edt_map,
  const nav_msgs::msg::MapMetaData & map_info,
  geometry_msgs::msg::PoseStamped & out_gradient_pose)
{
  MapIndex idx;
  float val;
  if (!getMinEDT(robot_pose, edt_map, map_info, idx, val)) {
    return false;
  }
  return getGrad(edt_map, map_info, idx, out_gradient_pose);
}

bool EdtGradientEstimator::calculate(
  const geometry_msgs::msg::Pose & robot_pose,
  const cv::Mat & edt_map,
  const nav2_costmap_2d::Costmap2D * costmap,
  geometry_msgs::msg::PoseStamped & out_gradient_pose)
{
  MapIndex idx;
  float val;
  if (!getMinEDT(robot_pose, edt_map, costmap, idx, val)) {
    return false;
  }
  return getGrad(edt_map, costmap, idx, out_gradient_pose);
}

} // namespace edt_gradient_estimator