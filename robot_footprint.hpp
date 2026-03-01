#ifndef EDT_MONITOR__ROBOT_FOOTPRINT_HPP_
#define EDT_MONITOR__ROBOT_FOOTPRINT_HPP_

#include <vector>
#include <cmath>
#include <algorithm>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace edt_monitor {

struct MapIndex {
    int x;
    int y;
};

/**
 * @brief Abstract base class for Robot Footprint.
 * Inherit from this to implement complex shapes (e.g., PolygonFootprint).
 */
class RobotFootprint {
public:
    virtual ~RobotFootprint() = default;

    /**
     * @brief Get the map indices that fall within the robot's footprint.
     * @param robot_pose The current pose of the robot in the map frame.
     * @param map The EDT map (OccupancyGrid).
     * @param[out] out_indices Vector to populate with indices. Cleared at start of function.
     */
    virtual void getIndices(
        const geometry_msgs::msg::Pose& robot_pose,
        const nav_msgs::msg::OccupancyGrid& map,
        std::vector<MapIndex>& out_indices) const = 0;
};

/**
 * @brief Circular implementation of the robot footprint.
 */
class CircularFootprint : public RobotFootprint {
public:
    explicit CircularFootprint(double radius) : radius_(radius), res_(-1.0) {}

    void getIndices(
        const geometry_msgs::msg::Pose& robot_pose,
        const nav_msgs::msg::OccupancyGrid& map,
        std::vector<MapIndex>& out_indices) const override {
        
        out_indices.clear();

        if (map.info.resolution != res_) {
            res_ = map.info.resolution;
            r_cells_ = radius_ / res_;
            n_steps_ = static_cast<int>(std::ceil(2 * M_PI * r_cells_ * 2.0));
            if (n_steps_ < 8) n_steps_ = 8;
        }
        if (res_ <= 0.0) return;

        origin_x_ = map.info.origin.position.x;
        origin_y_ = map.info.origin.position.y;
        width_ = map.info.width;
        height_ = map.info.height;

        // 1. robot_pose.position is in the Map's frame (e.g., "odom").
        // 2. origin is the position of cell (0,0) of the EDT in the Odom's frame.
        // 3. (robot - origin) gives the vector from the grid corner to the robot in meters.
        // 4. Divide by resolution to get continuous grid coordinates (cells).
        rx_grid_ = (robot_pose.position.x - origin_x_) / res_;
        ry_grid_ = (robot_pose.position.y - origin_y_) / res_;

        if (out_indices.capacity() < static_cast<size_t>(n_steps_)) {
            out_indices.reserve(n_steps_);
        }

        // Start at theta = pi/2 (top) and rotate counter-clockwise
        // We iterate 0 to n_steps to close the loop
        for (int i = 0; i <= n_steps_; ++i) {
            // Start at 90 degrees (Top of circle relative to grid)
            theta_ = M_PI_2 + (2.0 * M_PI * i / n_steps_);
            
            x_ = static_cast<int>(std::floor(rx_grid_ + r_cells_ * std::cos(theta_)));
            y_ = static_cast<int>(std::floor(ry_grid_ + r_cells_ * std::sin(theta_)));

            // Check bounds
            if (x_ >= 0 && x_ < width_ && y_ >= 0 && y_ < height_) {
                // Avoid sequential duplicates
                if (out_indices.empty() || out_indices.back().x != x_ || out_indices.back().y != y_) {
                    out_indices.push_back({x_, y_});
                }
            }
        }

        // Remove the last point if it's a duplicate of the first (loop closure)
        if (out_indices.size() > 1 && 
            out_indices.back().x == out_indices.front().x && 
            out_indices.back().y == out_indices.front().y) {
            out_indices.pop_back();
        }
    }

private:
    double radius_;
    mutable double res_;
    mutable double origin_x_;
    mutable double origin_y_;
    mutable int width_;
    mutable int height_;
    mutable double rx_grid_;
    mutable double ry_grid_;
    mutable double r_cells_;
    mutable int n_steps_;
    mutable double theta_;
    mutable int x_;
    mutable int y_;
};

} // namespace edt_monitor

#endif  // EDT_MONITOR__ROBOT_FOOTPRINT_HPP_