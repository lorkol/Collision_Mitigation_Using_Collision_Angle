#ifndef COLLISION_ANGLE_MITIGATION__EDT_GRADIENT_ESTIMATOR_HPP_
#define COLLISION_ANGLE_MITIGATION__EDT_GRADIENT_ESTIMATOR_HPP_

#include <memory>
#include <vector>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "opencv2/opencv.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "robot_footprint.hpp"

using namespace collision_angle_mitigation;
namespace edt_gradient_estimator
{

// Helper to safely access map data with bounds checking
double getMapValue(int ix, int iy, const cv::Mat & edt_map);


class EdtGradientEstimator
{
public:
  explicit EdtGradientEstimator(double robot_radius);
  ~EdtGradientEstimator();

  /**
   * @brief Calculates the pose on the footprint with the minimum EDT value and its gradient.
   * @param robot_pose The current robot pose in the map frame.
   * @param edt_map The EDT map data (float).
   * @param map_info The map metadata (resolution, origin, size).
   * @param out_gradient_pose The resulting pose (position = min EDT point, orientation = gradient direction).
   * @return true if successful, false if map is invalid or robot is off-map.
   */
  bool calculate(const geometry_msgs::msg::Pose & robot_pose, 
                 const cv::Mat & edt_map,
                 const nav_msgs::msg::MapMetaData & map_info,
                 geometry_msgs::msg::PoseStamped & out_gradient_pose);

  /**
   * @brief Finds the point on the footprint with the minimum EDT value.
   */
  bool getMinEDT(const geometry_msgs::msg::Pose & robot_pose, 
                 const cv::Mat & edt_map,
                 const nav_msgs::msg::MapMetaData & map_info,
                 MapIndex & out_min_idx,
                 float & out_min_val);

  /**
   * @brief Calculates the gradient at the specified point.
   */
  bool getGrad(const cv::Mat & edt_map,
               const nav_msgs::msg::MapMetaData & map_info,
               const MapIndex & min_idx,
               geometry_msgs::msg::PoseStamped & out_gradient_pose);


private:
  std::unique_ptr<RobotFootprint> footprint_model_;
  std::vector<MapIndex> footprint_indices_;

  // Pre-allocated variables for calculation to avoid reallocation

float min_val_;                 ///< Minimum EDT value found under the footprint
  MapIndex min_idx_; ///< Map index (x, y) corresponding to the minimum EDT value
  float center_val_;              ///< EDT value at the minimum index (center for gradient check)
  int best_dx_;                   ///< X offset (-1, 0, 1) pointing to the steepest descent neighbor
  int best_dy_;                   ///< Y offset (-1, 0, 1) pointing to the steepest descent neighbor
  double max_slope_;              ///< Maximum slope found among neighbors
  int nx_;                        ///< Neighbor x index during iteration
  int ny_;                        ///< Neighbor y index during iteration
  float val_;                     ///< EDT value at the current neighbor
  double diff_;                   ///< Difference between center value and neighbor value
  double dist_;                   ///< Distance to the neighbor (1.0 or sqrt(2))
  double slope_;                  ///< Calculated slope to the neighbor
  double yaw_;                    ///< Calculated yaw angle of the gradient
  tf2::Quaternion q_;             ///< Quaternion representing the gradient orientation

};

} // namespace edt_gradient_estimator

#endif // COLLISION_ANGLE_MITIGATION__EDT_GRADIENT_ESTIMATOR_HPP_