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

#ifndef NAV2_COSTMAP_2D__EDT_PUBLISHER_HPP_
#define NAV2_COSTMAP_2D__EDT_PUBLISHER_HPP_

#include <string>
#include <memory>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "opencv2/opencv.hpp"

namespace nav2_costmap_2d
{

class EDTPublisher
{
public:
  EDTPublisher(
    const nav2_util::LifecycleNode::WeakPtr & parent,
    Costmap2D * costmap,
    const std::string & global_frame,
    const std::string & topic_name);

  ~EDTPublisher();

  void on_activate();

  void on_deactivate();

  void publishEDT();

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  Costmap2D * costmap_;
  std::string global_frame_;
  std::string topic_name_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr edt_pub_;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__EDT_PUBLISHER_HPP_
