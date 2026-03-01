#include "robot_footprint.hpp"

#include <memory>
#include <string>
#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace edt_monitor {

class EdtMonitorNode : public rclcpp::Node {
public:
    EdtMonitorNode() : Node("edt_monitor_node") {
        // Parameters
        this->declare_parameter("robot_radius", 0.17); // TODO see how I can get this from the parameters file
        this->declare_parameter("base_frame", "base_link");
        this->declare_parameter("edt_topic", "edt_map");
        this->declare_parameter("robot_pose_topic", "amcl_pose"); // TODO Make this more generic or use TF only
        
        double radius = this->get_parameter("robot_radius").as_double();
        base_frame_ = this->get_parameter("base_frame").as_string();
        std::string edt_topic = this->get_parameter("edt_topic").as_string();
        std::string robot_pose_topic = this->get_parameter("robot_pose_topic").as_string();

        // Initialize footprint model (Polymorphic)
        footprint_model_ = std::make_unique<CircularFootprint>(radius);

        // Subscribers and Publishers
        edt_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            edt_topic, 1, std::bind(&EdtMonitorNode::edtCallback, this, std::placeholders::_1));
        
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            robot_pose_topic, 1, std::bind(&EdtMonitorNode::poseCallback, this, std::placeholders::_1));
        
        min_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("min_edt_pose", 10);

        // TF Buffer
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "EDT Monitor Node started. Listening to %s and %s", edt_topic.c_str(), robot_pose_topic.c_str());
    }

private:
    void edtCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        last_map_ = msg;
    }

    void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr /*msg*/) {
        get_EDT_grad();
    }

    void get_EDT_grad() {
        if (!last_map_) {
            return;
        }

        // Update cached map info if the map pointer has changed
        if (last_map_ != cached_map_) {
            // Only update metadata if it actually changed
            if (cached_map_ == nullptr){
                map_res_ = last_map_->info.resolution;
                map_width_ = static_cast<int>(last_map_->info.width);
                map_height_ = static_cast<int>(last_map_->info.height);
            }
            else if (cached_map_ == nullptr ||
                last_map_->info.origin.position.x != map_origin_x_ ||
                last_map_->info.origin.position.y != map_origin_y_) {
                map_origin_x_ = last_map_->info.origin.position.x;
                map_origin_y_ = last_map_->info.origin.position.y;
            }
            cached_map_ = last_map_;
            
            // Update message headers to match the current map
            pose_msg_.header = cached_map_->header;
        }

        // 1. Get Robot Pose in Map Frame
        try {
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                cached_map_->header.frame_id, base_frame_, tf2::TimePointZero);
            
            robot_pose_.position.x = transform.transform.translation.x;
            robot_pose_.position.y = transform.transform.translation.y;
            robot_pose_.position.z = transform.transform.translation.z;
            robot_pose_.orientation = transform.transform.rotation;
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                "Could not transform robot pose: %s", ex.what());
            return;
        }

        // 2. Get indices under footprint
        footprint_model_->getIndices(robot_pose_, *cached_map_, footprint_indices_);
        if (footprint_indices_.empty()) return;

        // 3. Find point with Minimum EDT value
        min_val_ = std::numeric_limits<int>::max();
        min_idx_ = {-1, -1};

        const auto& data = cached_map_->data;
        for (const MapIndex& idx : footprint_indices_) {
            index_ = idx.y * map_width_ + idx.x;
            int8_t val = data[index_];
            // Assuming -1 is unknown/invalid, skip it. 
            // TODO If the EDT map uses -1 for something else, adjust this check.
            if (val != -1 && val < min_val_) {
                min_val_ = val;
                min_idx_ = idx;
            }
        }

        if (min_idx_.x != -1) {
            publishResult();
        }
    }

    void publishResult() {
        // Calculate World Point using cached map info
        wx_ = map_origin_x_ + (min_idx_.x + 0.5) * map_res_;
        wy_ = map_origin_y_ + (min_idx_.y + 0.5) * map_res_;

        // Calculate Gradient using Central Difference
        // Gradient = (dVal/dx, dVal/dy)
        
        center_val_ = getMapValue(min_idx_.x, min_idx_.y);
        best_dx_ = 0;
        best_dy_ = 0;
        max_slope_ = 0.0;

        // Check 8 neighbors (including diagonals)
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                if (dx == 0 && dy == 0) continue;

                val_ = getMapValue(min_idx_.x + dx, min_idx_.y + dy);
                if (val_ == -1.0) continue;

                // "0 if the pixel itself is the minimum" implies we look for descent (lower values)
                diff_ = center_val_ - val_;
                if (diff_ > 0) {
                    dist_ = (dx != 0 && dy != 0) ? 1.41421356 : 1.0;
                    slope_ = diff_ / dist_;
                    if (slope_ > max_slope_) {
                        max_slope_ = slope_;
                        best_dx_ = dx;
                        best_dy_ = dy;
                    }
                }
            }
        }

        // Populate PoseStamped with point and gradient orientation
        pose_msg_.pose.position.x = wx_;
        pose_msg_.pose.position.y = wy_;
        pose_msg_.pose.position.z = 0.0;

        yaw_ = std::atan2(static_cast<double>(best_dy_), static_cast<double>(best_dx_));
        q_.setRPY(0, 0, yaw_+ M_PI); // Pointing towards the direction of steepest descent
        pose_msg_.pose.orientation = tf2::toMsg(q_);

        min_pose_pub_->publish(pose_msg_);
    }

    // Helper to safely access map data with bounds checking
    double getMapValue(int ix, int iy) {
        // Check if coordinates are outside map boundaries
        if (ix < 0 || ix >= map_width_ || iy < 0 || iy >= map_height_) return -1.0;
        // Return the value at the calculated linear index
        return static_cast<double>(cached_map_->data[iy * map_width_ + ix]);
    }

    std::string base_frame_;
    std::unique_ptr<RobotFootprint> footprint_model_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr edt_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr min_pose_pub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    nav_msgs::msg::OccupancyGrid::SharedPtr last_map_;
    nav_msgs::msg::OccupancyGrid::SharedPtr cached_map_;
    std::vector<MapIndex> footprint_indices_;
    
    // Cached map info and reusable variables
    double map_res_{0.0};
    int map_width_{0};
    int map_height_{0};
    double map_origin_x_{0.0};
    double map_origin_y_{0.0};
    geometry_msgs::msg::Pose robot_pose_;
    MapIndex min_idx_;
    geometry_msgs::msg::PoseStamped pose_msg_;

    // Reusable calculation variables
    double wx_{0.0};
    double wy_{0.0};
    double center_val_{0.0};
    int index_;
    int min_val_;
    int best_dx_{0};
    int best_dy_{0};
    double max_slope_{0.0};
    double val_{0.0};
    double diff_{0.0};
    double dist_{0.0};
    double slope_{0.0};
    tf2::Quaternion q_;
    double yaw_;
};

} // namespace edt_monitor

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<edt_monitor::EdtMonitorNode>());
    rclcpp::shutdown();
    return 0;
}