#include "map_memory_core.hpp"
#include <algorithm>

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
  : logger_(logger),
    global_grid_(width_, std::vector<int8_t>(height_, -1)) {} // Initialize global map as unknown (-1)

double MapMemoryCore::extractYaw(const geometry_msgs::msg::Quaternion &q) const {
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

void MapMemoryCore::integrateCostmap(const nav_msgs::msg::OccupancyGrid &costmap, const nav_msgs::msg::Odometry &odom) {
  double robot_x = odom.pose.pose.position.x;
  double robot_y = odom.pose.pose.position.y;
  double robot_yaw = extractYaw(odom.pose.pose.orientation);

  int costmap_width = costmap.info.width;
  int costmap_height = costmap.info.height;
  double costmap_res = costmap.info.resolution;
  double costmap_ox = costmap.info.origin.position.x;
  double costmap_oy = costmap.info.origin.position.y;

  for (int ly = 0; ly < costmap_height; ++ly) {
    for (int lx = 0; lx < costmap_width; ++lx) {
      int8_t local_cost = costmap.data[ly * costmap_width + lx];
      if (local_cost < 0) continue; // Skip unknown cells

      // Convert local cell to local Cartesian coordinates (meters relative to robot frame)
      double local_x = costmap_ox + (lx + 0.5) * costmap_res;
      double local_y = costmap_oy + (ly + 0.5) * costmap_res;

      // Transform local coordinates to global world coordinates using robot pose
      double global_x = robot_x + (local_x * std::cos(robot_yaw) - local_y * std::sin(robot_yaw));
      double global_y = robot_y + (local_x * std::sin(robot_yaw) + local_y * std::cos(robot_yaw));

      // Convert global coordinates to global grid indices
      int gx = static_cast<int>((global_x - origin_x_) / resolution_);
      int gy = static_cast<int>((global_y - origin_y_) / resolution_);

      if (gx >= 0 && gx < width_ && gy >= 0 && gy < height_) {
        // Fuse cost: prioritize high costs over low/unknown costs
        if (global_grid_[gx][gy] < 0) {
          global_grid_[gx][gy] = local_cost;
        } else {
          global_grid_[gx][gy] = std::max(global_grid_[gx][gy], local_cost);
        }
      }
    }
  }
}

nav_msgs::msg::OccupancyGrid MapMemoryCore::getGlobalMap() const {
  nav_msgs::msg::OccupancyGrid map_msg;
  map_msg.header.frame_id = "sim_world"; // Global reference frame

  map_msg.info.resolution = resolution_;
  map_msg.info.width = width_;
  map_msg.info.height = height_;
  map_msg.info.origin.position.x = origin_x_;
  map_msg.info.origin.position.y = origin_y_;
  map_msg.info.origin.position.z = 0.0;
  map_msg.info.origin.orientation.w = 1.0;

  map_msg.data.resize(width_ * height_);
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      map_msg.data[y * width_ + x] = global_grid_[x][y];
    }
  }

  return map_msg;
}

}
