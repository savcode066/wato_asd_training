#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode()
: Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

  grid_.assign(GRID_WIDTH_, std::vector<int8_t>(GRID_HEIGHT_, EMPTY_));
}

void CostmapNode::initializeCostmap() {
  for (auto &row : grid_) {
    std::fill(row.begin(), row.end(), EMPTY_);
  }
}

void CostmapNode::convertToGrid(double range, double angle, int &x_grid, int &y_grid) {
  double x = range * std::cos(angle);
  double y = range * std::sin(angle);

  // Shift so the robot (0, 0 in the laser frame) lands on the center cell,
  // since obstacles can be behind/left of the sensor (negative x/y).
  x_grid = static_cast<int>(std::floor(x / RESOLUTION_)) + GRID_WIDTH_ / 2;
  y_grid = static_cast<int>(std::floor(y / RESOLUTION_)) + GRID_HEIGHT_ / 2;
}

void CostmapNode::markObstacle(int x_grid, int y_grid) {
  if (x_grid < 0 || x_grid >= GRID_WIDTH_ || y_grid < 0 || y_grid >= GRID_HEIGHT_) {
    return;
  }
  grid_[x_grid][y_grid] = OCCUPIED_;
}

void CostmapNode::inflateObstacles() {
  int inflation_cells = static_cast<int>(INFLATION_RADIUS_ / RESOLUTION_);

  // Snapshot the pre-inflation obstacles so newly-inflated cells don't get
  // treated as obstacle sources within the same pass.
  const auto obstacles = grid_;

  for (int i = 0; i < GRID_WIDTH_; ++i) {
    for (int j = 0; j < GRID_HEIGHT_; ++j) {
      if (obstacles[i][j] != OCCUPIED_) continue;

      for (int di = -inflation_cells; di <= inflation_cells; ++di) {
        for (int dj = -inflation_cells; dj <= inflation_cells; ++dj) {
          int ni = i + di;
          int nj = j + dj;
          if (ni < 0 || ni >= GRID_WIDTH_ || nj < 0 || nj >= GRID_HEIGHT_) continue;

          double distance = std::sqrt(static_cast<double>(di * di + dj * dj)) * RESOLUTION_;
          if (distance > INFLATION_RADIUS_) continue;

          int cost = static_cast<int>(MAX_INFLATION_ * (1.0 - (distance / INFLATION_RADIUS_)));
          if (cost > grid_[ni][nj]) {
            grid_[ni][nj] = static_cast<int8_t>(cost);
          }
        }
      }
    }
  }
}

void CostmapNode::publishCostmap(const std_msgs::msg::Header &header) {
  nav_msgs::msg::OccupancyGrid msg;
  msg.header = header;

  msg.info.resolution = static_cast<float>(RESOLUTION_);
  msg.info.width = GRID_WIDTH_;
  msg.info.height = GRID_HEIGHT_;
  msg.info.origin.position.x = -(GRID_WIDTH_ * RESOLUTION_) / 2.0;
  msg.info.origin.position.y = -(GRID_HEIGHT_ * RESOLUTION_) / 2.0;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.w = 1.0;

  msg.data.resize(GRID_WIDTH_ * GRID_HEIGHT_);
  for (int i = 0; i < GRID_WIDTH_; ++i) {
    for (int j = 0; j < GRID_HEIGHT_; ++j) {
      // OccupancyGrid data is row-major with x varying fastest.
      msg.data[j * GRID_WIDTH_ + i] = grid_[i][j];
    }
  }

  occupancy_grid_pub_->publish(msg);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  initializeCostmap();

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double angle = scan->angle_min + i * scan->angle_increment;
    double range = scan->ranges[i];
    if (range < scan->range_max && range > scan->range_min) {
      int x_grid, y_grid;
      convertToGrid(range, angle, x_grid, y_grid);
      markObstacle(x_grid, y_grid);
    }
  }

  inflateObstacles();
  publishCostmap(scan->header);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
