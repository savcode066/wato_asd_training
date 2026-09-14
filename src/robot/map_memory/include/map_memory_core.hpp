#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class MapMemoryCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    // Resets the global map to all-unknown cells with the given geometry.
    void initializeGlobalMap(double resolution, int width, int height, double origin_x, double origin_y);

    // Transforms a robot-relative costmap into the global frame using the robot's
    // pose and fuses it into the global map: known cells overwrite, unknown cells
    // leave the existing global value untouched.
    void integrateCostmap(const nav_msgs::msg::OccupancyGrid &costmap,
                          double robot_x, double robot_y, double robot_yaw);

    const nav_msgs::msg::OccupancyGrid &globalMap() const { return global_map_; }

  private:
    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid global_map_;
};

}

#endif
