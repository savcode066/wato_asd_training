#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    robot::MapMemoryCore map_memory_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void updateMap();

    static double yawFromQuaternion(const geometry_msgs::msg::Quaternion &q);

    // Global map geometry, sized to comfortably hold an extended run without
    // needing to grow as the robot explores.
    static constexpr double RESOLUTION_ = 0.1;          // meters per cell
    static constexpr int WIDTH_ = 300;                  // cells (30m at 0.1 m/cell)
    static constexpr int HEIGHT_ = 300;                 // cells (30m at 0.1 m/cell)
    static constexpr double DISTANCE_THRESHOLD_ = 1.5;  // meters the robot must move before fusing a new costmap

    nav_msgs::msg::OccupancyGrid latest_costmap_;
    bool costmap_received_ = false;

    double last_x_ = 0.0;
    double last_y_ = 0.0;
    bool has_last_position_ = false;

    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_yaw_ = 0.0;

    bool should_update_map_ = false;
};

#endif
