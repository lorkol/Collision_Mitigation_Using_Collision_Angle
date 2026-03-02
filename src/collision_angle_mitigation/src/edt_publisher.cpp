// Copyright (c) 2023, Your Name
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

#include "edt_publisher.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <algorithm>
#include <memory>
#include "collision_angle_mitigation/msg/float_edm.hpp"

namespace nav2_costmap_2d
{

EDTPublisher::EDTPublisher(
  const nav2_util::LifecycleNode::WeakPtr & parent,
  Costmap2D * costmap,
  const std::string & global_frame,
  const std::string & topic_name): parent_(parent), costmap_(costmap), global_frame_(global_frame), topic_name_(topic_name)
{
  auto node = parent.lock();
  clock_ = node->get_clock();

  auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  edt_pub_ = node->create_publisher<collision_angle_mitigation::msg::FloatEDM>(topic_name_, custom_qos);
}

EDTPublisher::~EDTPublisher()
{
}

void EDTPublisher::on_activate()
{
  edt_pub_->on_activate();
}

void EDTPublisher::on_deactivate()
{
  edt_pub_->on_deactivate();
}

void EDTPublisher::publishEDT()
{
  if (edt_pub_->get_subscription_count() == 0) {
    return;
  }

  std::unique_lock<Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

  unsigned int width = costmap_->getSizeInCellsX();
  unsigned int height = costmap_->getSizeInCellsY();

  if (width == 0 || height == 0) {
    return;
  }

  // 1. Create binary map from costmap
  // Create a cv::Mat header for the costmap data without copying.
  cv::Mat costmap_mat(height, width, CV_8UC1, const_cast<unsigned char *>(costmap_->getCharMap()));
  cv::Mat binary_map;

  // Use OpenCV's 'compare' to efficiently create a binary map.
  // Cells with cost < LETHAL_OBSTACLE become 255 (free), and others become 0 (obstacle).
  cv::compare(costmap_mat, nav2_costmap_2d::LETHAL_OBSTACLE, binary_map, cv::CMP_LT);

  // 2. Create FloatEDM message
  auto edt_msg = std::make_unique<collision_angle_mitigation::msg::FloatEDM>();

  edt_msg->header.stamp = clock_->now();
  edt_msg->header.frame_id = global_frame_;
  edt_msg->info.map_load_time = edt_msg->header.stamp;
  edt_msg->info.resolution = costmap_->getResolution();
  edt_msg->info.width = width;
  edt_msg->info.height = height;

  // The collision_angle_mitigation::msg::FloatEDM is always axis-aligned WITH ODOM, so we can get the origin directly.
  //This isnt where the robot is, it's the bottom-left corner of the map relative to odom. The robot is centered in the costmap
  edt_msg->info.origin.position.x = costmap_->getOriginX();
  edt_msg->info.origin.position.y = costmap_->getOriginY();
  edt_msg->info.origin.position.z = 0.0;
  edt_msg->info.origin.orientation.w = 1.0;

  edt_msg->data.resize(width * height);

  // 3. Compute EDT directly into the message buffer
  // Create a cv::Mat header that points directly to the message's data buffer
  cv::Mat edt_map(height, width, CV_32F, edt_msg->data.data());
  cv::distanceTransform(binary_map, edt_map, cv::DIST_L2, cv::DIST_MASK_PRECISE);

  edt_pub_->publish(std::move(edt_msg));
}

}  // namespace nav2_costmap_2d
