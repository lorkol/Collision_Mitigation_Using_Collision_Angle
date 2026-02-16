#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "opencv2/opencv.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

class EdtPublisherNode : public rclcpp::Node
{
public:
  EdtPublisherNode() : Node("edt_publisher_node")
  {
    // Parameters
    const auto costmap_topic = this->declare_parameter<std::string>(
      "costmap_topic", "/global_costmap/costmap_raw");
    const auto edt_map_topic = this->declare_parameter<std::string>("edt_map_topic", "/edt_map");
    obstacle_threshold_ = this->declare_parameter<int>(
      "obstacle_threshold", nav2_costmap_2d::LETHAL_OBSTACLE);

    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&EdtPublisherNode::parametersCallback, this, std::placeholders::_1));

    // QoS for map topics should match the costmap publisher (transient_local)
    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

    // Subscriber to the raw costmap.
    costmap_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>(
      costmap_topic,
      map_qos,
      std::bind(&EdtPublisherNode::costmapCallback, this, std::placeholders::_1));

    // Publisher for the EDT map
    edt_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(edt_map_topic, map_qos);

    RCLCPP_INFO(this->get_logger(), "EDT Publisher Node has been started.");
    RCLCPP_INFO(this->get_logger(), "Subscribing to raw costmap on topic: %s", costmap_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing EDT on topic: %s", edt_map_topic.c_str());
  }

private:
  void costmapCallback(const nav2_msgs::msg::Costmap::ConstSharedPtr msg)
  {
    RCLCPP_DEBUG(this->get_logger(), "Received raw costmap");

    const unsigned int width = msg->metadata.size_x;
    const unsigned int height = msg->metadata.size_y;

    if (width == 0 || height == 0) {
      RCLCPP_WARN(this->get_logger(), "Received an empty costmap.");
      return;
    }

    // 1. Create a binary map from the raw costmap data for OpenCV
    // Re-use cv::Mat to avoid reallocation if size is the same
    if (binary_map_.size().width != static_cast<int>(width) ||
      binary_map_.size().height != static_cast<int>(height))
    {
      binary_map_ = cv::Mat(height, width, CV_8UC1);
    }

    for (unsigned int r = 0; r < height; ++r) {
      for (unsigned int c = 0; c < width; ++c) {
        const unsigned char cost = msg->data[r * width + c];
        if (cost >= obstacle_threshold_) {
          binary_map_.at<unsigned char>(r, c) = 0;   // Obstacle for distanceTransform
        } else {
          binary_map_.at<unsigned char>(r, c) = 255; // Free space
        }
      }
    }

    // 2. Compute the Exact Distance Transform (EDT)
    // Fills edt_map_ with the distance to the nearest obstacle in pixels
    cv::distanceTransform(binary_map_, edt_map_, cv::DIST_L2, cv::DIST_MASK_PRECISE);

    // 3. Create and publish the EDT as a nav_msgs::msg::OccupancyGrid
    auto edt_msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();

    edt_msg->header = msg->header;
    edt_msg->info.map_load_time = this->now();
    edt_msg->info.resolution = msg->metadata.resolution;
    edt_msg->info.width = width;
    edt_msg->info.height = height;
    edt_msg->info.origin = msg->metadata.origin;

    edt_msg->data.resize(width * height);

    // 4. Fill the OccupancyGrid data with EDT values (distance in cells, capped at 500).
    for (unsigned int j = 0; j < height; ++j) {
      for (unsigned int i = 0; i < width; ++i) {
        const float dist_in_pixels = edt_map_.at<float>(j, i);
        edt_msg->data[j * width + i] = static_cast<int8_t>(std::min(127.0f, dist_in_pixels));
      }
    }

    edt_pub_->publish(std::move(edt_msg));
    RCLCPP_DEBUG(this->get_logger(), "Published EDT map.");
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & parameter : parameters) {
      if (parameter.get_name() == "obstacle_threshold") {
        obstacle_threshold_ = parameter.as_int();
        RCLCPP_INFO(this->get_logger(), "Obstacle threshold updated to: %d", obstacle_threshold_);
      }
    }
    return result;
  }

  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr edt_pub_;
  int obstacle_threshold_;
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  cv::Mat binary_map_, edt_map_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EdtPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
