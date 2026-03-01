#include "edt_monitor/robot_footprint.hpp"

#include <cmath>
#include <algorithm>

namespace edt_monitor {

CircularFootprint::CircularFootprint(double radius) : radius_(radius), res_(-1.0) {}

void CircularFootprint::getIndices(
    const geometry_msgs::msg::Pose& robot_pose,
    const nav_msgs::msg::OccupancyGrid& map,
    std::vector<MapIndex>& out_indices) const {
    
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

} // namespace edt_monitor