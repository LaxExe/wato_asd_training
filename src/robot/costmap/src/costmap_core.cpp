#include "costmap_core.hpp"
#include <algorithm>

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

bool CostmapCore::worldToGrid(double wx, double wy, int &gx, int &gy) const {
  if (wx < origin_x_ || wy < origin_y_) {
    return false;
  }
  gx = static_cast<int>((wx - origin_x_) / resolution_);
  gy = static_cast<int>((wy - origin_y_) / resolution_);
  return (gx >= 0 && gx < width_ && gy >= 0 && gy < height_);
}

void CostmapCore::markObstacle(std::vector<std::vector<int>> &grid, int gx, int gy) {
  if (gx >= 0 && gx < width_ && gy >= 0 && gy < height_) {
    grid[gx][gy] = max_cost_;
  }
}

void CostmapCore::inflateObstacles(std::vector<std::vector<int>> &grid) {
  int inflation_cells = static_cast<int>(std::ceil(inflation_radius_ / resolution_));
  std::vector<std::vector<int>> inflated_grid = grid;

  for (int x = 0; x < width_; ++x) {
    for (int y = 0; y < height_; ++y) {
      if (grid[x][y] == max_cost_) {
        for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
          for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
            int nx = x + dx;
            int ny = y + dy;

            if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
              double dist = std::sqrt(dx * dx + dy * dy) * resolution_;
              if (dist <= inflation_radius_) {
                int cost = static_cast<int>(max_cost_ * (1.0 - (dist / inflation_radius_)));
                inflated_grid[nx][ny] = std::max(inflated_grid[nx][ny], cost);
              }
            }
          }
        }
      }
    }
  }

  grid = inflated_grid;
}

nav_msgs::msg::OccupancyGrid CostmapCore::processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  std::vector<std::vector<int>> grid(width_, std::vector<int>(height_, 0));

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double range = scan->ranges[i];
    if (range >= scan->range_min && range <= scan->range_max) {
      double angle = scan->angle_min + i * scan->angle_increment;
      double x = range * std::cos(angle);
      double y = range * std::sin(angle);

      int gx, gy;
      if (worldToGrid(x, y, gx, gy)) {
        markObstacle(grid, gx, gy);
      }
    }
  }

  inflateObstacles(grid);

  nav_msgs::msg::OccupancyGrid occupancy_grid;
  occupancy_grid.header = scan->header;
  occupancy_grid.header.frame_id = "robot/base_link";

  occupancy_grid.info.resolution = resolution_;
  occupancy_grid.info.width = width_;
  occupancy_grid.info.height = height_;
  occupancy_grid.info.origin.position.x = origin_x_;
  occupancy_grid.info.origin.position.y = origin_y_;
  occupancy_grid.info.origin.position.z = 0.0;
  occupancy_grid.info.origin.orientation.w = 1.0;

  occupancy_grid.data.resize(width_ * height_);
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      occupancy_grid.data[y * width_ + x] = static_cast<int8_t>(grid[x][y]);
    }
  }

  return occupancy_grid;
}

}