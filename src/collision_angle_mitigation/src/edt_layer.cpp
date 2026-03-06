#include "edt_layer.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(collision_angle_mitigation::EdtLayer, nav2_costmap_2d::Layer)

namespace collision_angle_mitigation
{

EdtLayer::EdtLayer() {}
EdtLayer::~EdtLayer() {}

void EdtLayer::onInitialize()
{
  auto node = node_.lock(); // Get a shared pointer to the node after locking it
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("obstacle_threshold", rclcpp::ParameterValue(100));
  declareParameter("edt_topic", rclcpp::ParameterValue("/edt_map"));

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "obstacle_threshold", obstacle_threshold_);
  
  std::string topic;
  node->get_parameter(name_ + "." + "edt_topic", topic);

  // Define custom_qos before using it
  auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  edt_pub_ = node->create_publisher<collision_angle_mitigation::msg::FloatEDM>(
      topic, custom_qos);
      
  RCLCPP_INFO(logger_, "EDT Layer initialized successfully. Publishing to: %s", topic.c_str());
  current_ = true;
}

void EdtLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * /*min_x*/, double * /*min_y*/, double * /*max_x*/, double * /*max_y*/)
{
  if (!enabled_) return;
  // We don't need to expand bounds because we don't write to the master_grid costmap,
  // we only read from it to generate our internal float map.
}

void EdtLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/)
{
  if (!enabled_) return;

  unsigned int size_x = master_grid.getSizeInCellsX();
  unsigned int size_y = master_grid.getSizeInCellsY();
  unsigned char* char_map = master_grid.getCharMap();

  if (size_x == 0 || size_y == 0) {
    return;
  }

  // 1. Wrap master grid in OpenCV Mat (No copy yet)
  cv::Mat master_mat(size_y, size_x, CV_8UC1, char_map);

  // 2. Create binary map
  // Pixels < threshold are "free" (255), others are obstacles (0)
  // distanceTransform calculates distance to nearest ZERO pixel.
  cv::compare(master_mat, obstacle_threshold_, binary_map_, cv::CMP_LT);

  // 3. Compute EDT into a temporary buffer
  // Optimization: If the scratch buffer is currently being read by an external node 
  // (refcount > 1), release it so OpenCV allocates a fresh buffer.
  // Note: edt_scratch_ holds the *previous* cycle's edt_float_grid_ data due to the swap below.
  // So if an external node called getEdt(), they are holding a ref to what is now edt_scratch_.
  // If refcount == 1, we reuse the existing memory (Zero Allocation).
  if (edt_scratch_.u && edt_scratch_.u->refcount > 1) {
    edt_scratch_.release();
  }
  cv::distanceTransform(binary_map_, edt_scratch_, cv::DIST_L2, cv::DIST_MASK_PRECISE);

  // 4. Publish for Visualization (Optional)
  if (edt_pub_->get_subscription_count() > 0) {
    auto msg = std::make_unique<collision_angle_mitigation::msg::FloatEDM>();
    msg->header.stamp = clock_->now();
    msg->header.frame_id = layered_costmap_->getGlobalFrameID();
    msg->info.resolution = master_grid.getResolution();
    msg->info.width = size_x;
    msg->info.height = size_y;
    msg->info.origin.position.x = master_grid.getOriginX();
    msg->info.origin.position.y = master_grid.getOriginY();
    
    msg->data.resize(size_x * size_y);
    
    if (edt_scratch_.isContinuous()) {
      std::memcpy(msg->data.data(), edt_scratch_.data, size_x * size_y * sizeof(float));
    } else {
      for (int i = 0; i < edt_scratch_.rows; ++i) {
        const float* row_ptr = edt_scratch_.ptr<float>(i);
        std::memcpy(&msg->data[i * size_x], row_ptr, size_x * sizeof(float));
      }
    }
    edt_pub_->publish(std::move(msg));
  }

  // 5. Write to Shared Memory (Thread Safe)
  {
    std::lock_guard<std::mutex> lock(edt_mutex_);
    // Swap buffers: edt_float_grid_ gets the new data, edt_scratch_ gets the old data.
    // In the next cycle, edt_scratch_ (holding the old data) will be checked for external references.
    std::swap(edt_float_grid_, edt_scratch_);
  }
}

void EdtLayer::activate()
{
  edt_pub_->on_activate();
  RCLCPP_INFO(logger_, "EDT Layer activated");
  current_ = true;
}

void EdtLayer::deactivate()
{
  edt_pub_->on_deactivate();
  current_ = false;
}

cv::Mat EdtLayer::getEdt()
{
  std::lock_guard<std::mutex> lock(edt_mutex_);
  return edt_float_grid_; 
}

nav2_costmap_2d::Costmap2D* EdtLayer::getCostmap()
{
  return layered_costmap_->getCostmap();
}

void EdtLayer::reset()
{
  current_ = false;
}

} // namespace collision_angle_mitigation