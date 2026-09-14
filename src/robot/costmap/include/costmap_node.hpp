#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include <cstdint>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/header.hpp"

#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();

    // Callback for incoming laser scans
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

  private:
    robot::CostmapCore costmap_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;

    void initializeCostmap();
    void convertToGrid(double range, double angle, int &x_grid, int &y_grid);
    void markObstacle(int x_grid, int y_grid);
    void inflateObstacles();
    void publishCostmap(const std_msgs::msg::Header &header);

    // Grid parameters. The robot sits at the center of the grid so that
    // obstacles behind/left of the sensor (negative x/y) still map to valid cells.
    static constexpr double RESOLUTION_ = 0.1;    // meters per cell
    static constexpr int GRID_WIDTH_ = 100;        // cells (10m at 0.1 m/cell)
    static constexpr int GRID_HEIGHT_ = 100;       // cells (10m at 0.1 m/cell)
    static constexpr int8_t EMPTY_ = 0;
    static constexpr int8_t OCCUPIED_ = 100;
    static constexpr double INFLATION_RADIUS_ = 1.0; // meters
    static constexpr int8_t MAX_INFLATION_ = 100;

    std::vector<std::vector<int8_t>> grid_;
};

#endif
