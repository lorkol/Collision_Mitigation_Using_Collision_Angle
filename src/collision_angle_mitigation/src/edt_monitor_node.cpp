#include "edt_gradient_estimator.hpp"

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

#include "collision_angle_mitigation/msg/float_edm.hpp"

using namespace collision_angle_mitigation;
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
        edt_estimator_ = std::make_unique<edt_gradient_estimator::EdtGradientEstimator>(radius);

        // Subscribers and Publishers
        edt_sub_ = this->create_subscription<msg::FloatEDM>(
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
    void edtCallback(const msg::FloatEDM::SharedPtr msg) {
        edt_map_ = msg;
    }

    // TODO: This is currently called when the robot moves, but will need to somehow move to the MPPI trajectories
    void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr /*msg*/) {
        get_EDT_grad();
    }

    void get_EDT_grad() {
        if (!edt_map_) {
            return;
        }

        // 1. Get Robot Pose in Map Frame
        try {
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                edt_map_->header.frame_id, base_frame_, tf2::TimePointZero);
            
            robot_pose_.position.x = transform.transform.translation.x;
            robot_pose_.position.y = transform.transform.translation.y;
            robot_pose_.position.z = transform.transform.translation.z;
            robot_pose_.orientation = transform.transform.rotation;
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                "Could not transform robot pose: %s", ex.what());
            return;
        }

        // 2. Use Estimator
        cv::Mat edt_mat(edt_map_->info.height, edt_map_->info.width, CV_32F, const_cast<float*>(edt_map_->data.data()));
        
        if (edt_estimator_->calculate(robot_pose_, edt_mat, edt_map_->info, pose_msg_)) {
            pose_msg_.header = edt_map_->header;
            min_pose_pub_->publish(pose_msg_);
        }
    }

    std::string base_frame_;
    std::unique_ptr<edt_gradient_estimator::EdtGradientEstimator> edt_estimator_;
    rclcpp::Subscription<msg::FloatEDM>::SharedPtr edt_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr min_pose_pub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    msg::FloatEDM::SharedPtr edt_map_;
    
    geometry_msgs::msg::Pose robot_pose_;
    geometry_msgs::msg::PoseStamped pose_msg_;
};

} // namespace edt_monitor

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<edt_monitor::EdtMonitorNode>());
    rclcpp::shutdown();
    return 0;
}