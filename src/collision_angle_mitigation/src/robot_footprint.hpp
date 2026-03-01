#ifndef EDT_MONITOR__ROBOT_FOOTPRINT_HPP_
#define EDT_MONITOR__ROBOT_FOOTPRINT_HPP_

#include <vector>
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
     * @param robot_pose The current pose of the robot in the local costmap frame.
     * @param map The local costmap (OccupancyGrid/EDT).
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
    explicit CircularFootprint(double radius);

    void getIndices(
        const geometry_msgs::msg::Pose& robot_pose,
        const nav_msgs::msg::OccupancyGrid& map,
        std::vector<MapIndex>& out_indices) const override;

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