#ifndef COLLISION_ANGLE_MITIGATION__EDT_LAYER_HPP_
#define COLLISION_ANGLE_MITIGATION__EDT_LAYER_HPP_

#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "collision_angle_mitigation/msg/float_edm.hpp"
#include "opencv2/opencv.hpp"

namespace collision_angle_mitigation
{

class EdtLayer : public nav2_costmap_2d::Layer
{
public:
  EdtLayer();
  virtual ~EdtLayer();

  virtual void onInitialize();
  
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y);

  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  virtual void activate() override;
  virtual void deactivate() override;

  virtual void reset();
  virtual bool isClearable() { return false; }

  /**
   * @brief Shared Memory Accessor
   * Call this from your Critic/Planner to get the latest high-precision float map.
   * Returns a deep copy to ensure thread safety during read operations.
   */
  cv::Mat getEdt();

private:
  cv::Mat edt_float_grid_;
  // Reusable buffer to avoid reallocations each cycle
  cv::Mat edt_scratch_;
  // Reusable buffer to avoid reallocations each cycle
  cv::Mat binary_map_;
  mutable std::mutex edt_mutex_; // Protects the shared float map
  int obstacle_threshold_;
  rclcpp_lifecycle::LifecyclePublisher<collision_angle_mitigation::msg::FloatEDM>::SharedPtr edt_pub_;
};

} // namespace collision_angle_mitigation

#endif
