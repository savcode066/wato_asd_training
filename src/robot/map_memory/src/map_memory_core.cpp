#include <cmath>

#include "map_memory_core.hpp"

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
  : logger_(logger) {}

void MapMemoryCore::initializeGlobalMap(double resolution, int width, int height, double origin_x, double origin_y) {
  global_map_.info.resolution = static_cast<float>(resolution);
  global_map_.info.width = width;
  global_map_.info.height = height;
  global_map_.info.origin.position.x = origin_x;
  global_map_.info.origin.position.y = origin_y;
  global_map_.info.origin.position.z = 0.0;
  global_map_.info.origin.orientation.w = 1.0;
  global_map_.data.assign(static_cast<size_t>(width) * height, -1);
}

void MapMemoryCore::integrateCostmap(const nav_msgs::msg::OccupancyGrid &costmap,
                                      double robot_x, double robot_y, double robot_yaw) {
  const double global_res = global_map_.info.resolution;
  const int global_width = static_cast<int>(global_map_.info.width);
  const int global_height = static_cast<int>(global_map_.info.height);
  const double global_origin_x = global_map_.info.origin.position.x;
  const double global_origin_y = global_map_.info.origin.position.y;

  const double local_res = costmap.info.resolution;
  const int local_width = static_cast<int>(costmap.info.width);
  const int local_height = static_cast<int>(costmap.info.height);
  const double local_origin_x = costmap.info.origin.position.x;
  const double local_origin_y = costmap.info.origin.position.y;

  const double cos_yaw = std::cos(robot_yaw);
  const double sin_yaw = std::sin(robot_yaw);

  for (int j = 0; j < local_height; ++j) {
    for (int i = 0; i < local_width; ++i) {
      int8_t value = costmap.data[static_cast<size_t>(j) * local_width + i];
      if (value < 0) continue;  // unknown local cell: retain whatever the global map already has

      // Local cell center, in the costmap's own robot-relative frame.
      double local_x = local_origin_x + (i + 0.5) * local_res;
      double local_y = local_origin_y + (j + 0.5) * local_res;

      // Rotate/translate into the global (world) frame using the robot's pose.
      double world_x = robot_x + local_x * cos_yaw - local_y * sin_yaw;
      double world_y = robot_y + local_x * sin_yaw + local_y * cos_yaw;

      int gi = static_cast<int>(std::floor((world_x - global_origin_x) / global_res));
      int gj = static_cast<int>(std::floor((world_y - global_origin_y) / global_res));

      if (gi < 0 || gi >= global_width || gj < 0 || gj >= global_height) continue;

      global_map_.data[static_cast<size_t>(gj) * global_width + gi] = value;
    }
  }
}

}
