#include "robot_footprint.hpp"

#include <cmath>
#include <algorithm>

namespace collision_angle_mitigation {

CircularFootprint::CircularFootprint(double radius) : radius_(radius), res_(-1.0) {}

void CircularFootprint::getIndices(
    const geometry_msgs::msg::Pose& robot_pose,
    const nav_msgs::msg::MapMetaData& map_info,
    std::vector<MapIndex>& out_indices) const {
    
    out_indices.clear();

    double res = map_info.resolution;
    if (res <= 0.0) return;

    double r_cells = radius_ / res;
    int n_steps = static_cast<int>(std::ceil(2 * M_PI * r_cells * 2.0));
    if (n_steps < 8) n_steps = 8;

    double origin_x = map_info.origin.position.x;
    double origin_y = map_info.origin.position.y;
    int width = map_info.width;
    int height = map_info.height;

    // 1. robot_pose.position is in the Map's frame (e.g., "odom").
    // 2. origin is the position of cell (0,0) of the EDT in the Odom's frame.
    // 3. (robot - origin) gives the vector from the grid corner to the robot in meters.
    // 4. Divide by resolution to get continuous grid coordinates (cells).
    double rx_grid = (robot_pose.position.x - origin_x) / res;
    double ry_grid = (robot_pose.position.y - origin_y) / res;

    if (out_indices.capacity() < static_cast<size_t>(n_steps)) {
        out_indices.reserve(n_steps);
    }

    // Start at theta = pi/2 (top) and rotate counter-clockwise
    // We iterate 0 to n_steps to close the loop
    for (int i = 0; i <= n_steps; ++i) {
        // Start at 90 degrees (Top of circle relative to grid)
        double theta = M_PI_2 + (2.0 * M_PI * i / n_steps);
        
        int x = static_cast<int>(std::round(rx_grid + r_cells * std::cos(theta)));
        int y = static_cast<int>(std::round(ry_grid + r_cells * std::sin(theta)));

        // Check bounds
        if (x >= 0 && x < width && y >= 0 && y < height) {
            // Avoid sequential duplicates
            if (out_indices.empty() || out_indices.back().x != x || out_indices.back().y != y) {
                out_indices.push_back({x, y});
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

void CircularFootprint::getIndices(
    const geometry_msgs::msg::Pose& robot_pose,
    const nav2_costmap_2d::Costmap2D& costmap,
    std::vector<MapIndex>& out_indices) const {
    
    out_indices.clear();

    double res = costmap.getResolution();
    if (res <= 0.0) return;

    double r_cells = radius_ / res;
    int n_steps = static_cast<int>(std::ceil(2 * M_PI * r_cells * 2.0));
    if (n_steps < 8) n_steps = 8;

    double origin_x = costmap.getOriginX();
    double origin_y = costmap.getOriginY();
    int width = costmap.getSizeInCellsX();
    int height = costmap.getSizeInCellsY();

    // 1. robot_pose.position is in the Map's frame (e.g., "odom").
    // 2. origin is the position of cell (0,0) of the EDT in the Odom's frame.
    // 3. (robot - origin) gives the vector from the grid corner to the robot in meters.
    // 4. Divide by resolution to get continuous grid coordinates (cells).
    double rx_grid = (robot_pose.position.x - origin_x) / res;
    double ry_grid = (robot_pose.position.y - origin_y) / res;

    if (out_indices.capacity() < static_cast<size_t>(n_steps)) {
        out_indices.reserve(n_steps);
    }

    // Start at theta = pi/2 (top) and rotate counter-clockwise
    // We iterate 0 to n_steps to close the loop
    for (int i = 0; i <= n_steps; ++i) {
        // Start at 90 degrees (Top of circle relative to grid)
        double theta = M_PI_2 + (2.0 * M_PI * i / n_steps);
        
        int x = static_cast<int>(std::round(rx_grid + r_cells * std::cos(theta)));
        int y = static_cast<int>(std::round(ry_grid + r_cells * std::sin(theta)));

        // Check bounds
        if (x >= 0 && x < width && y >= 0 && y < height) {
            // Avoid sequential duplicates
            if (out_indices.empty() || out_indices.back().x != x || out_indices.back().y != y) {
                out_indices.push_back({x, y});
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

} // namespace collision_angle_mitigation