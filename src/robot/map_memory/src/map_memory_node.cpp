#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode()
: Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  map_memory_.initializeGlobalMap(RESOLUTION_, WIDTH_, HEIGHT_,
                                   -(WIDTH_ * RESOLUTION_) / 2.0, -(HEIGHT_ * RESOLUTION_) / 2.0);

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  // Limits how often we bother re-fusing/re-publishing the map, independent
  // of how fast costmaps or odometry are arriving.
  timer_ = this->create_wall_timer(
    std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));
}

double MapMemoryNode::yawFromQuaternion(const geometry_msgs::msg::Quaternion &q) {
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = *msg;
  costmap_received_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  robot_yaw_ = yawFromQuaternion(msg->pose.pose.orientation);

  if (!has_last_position_) {
    last_x_ = robot_x_;
    last_y_ = robot_y_;
    has_last_position_ = true;
    return;
  }

  double distance = std::sqrt(std::pow(robot_x_ - last_x_, 2) + std::pow(robot_y_ - last_y_, 2));
  if (distance >= DISTANCE_THRESHOLD_) {
    last_x_ = robot_x_;
    last_y_ = robot_y_;
    should_update_map_ = true;
  }
}

void MapMemoryNode::updateMap() {
  if (!should_update_map_ || !costmap_received_) return;

  map_memory_.integrateCostmap(latest_costmap_, robot_x_, robot_y_, robot_yaw_);

  auto global_map = map_memory_.globalMap();
  global_map.header.stamp = this->now();
  global_map.header.frame_id = "map";
  map_pub_->publish(global_map);

  should_update_map_ = false;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
