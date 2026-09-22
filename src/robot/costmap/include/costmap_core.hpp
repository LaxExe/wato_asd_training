#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <vector>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class CostmapCore {
  public:
    explicit CostmapCore(const rclcpp::Logger& logger);

    // Converts incoming LaserScan into an OccupancyGrid message
    nav_msgs::msg::OccupancyGrid processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr scan);

  private:
    rclcpp::Logger logger_;

    // Grid Parameters
    double resolution_ = 0.1;   // 0.1 meters per cell
    int width_ = 200;           // 200 cells (20 meters total width)
    int height_ = 200;          // 200 cells (20 meters total height)
    double origin_x_ = -10.0;   // Robot centered at cell (100, 100)
    double origin_y_ = -10.0;

    // Inflation Parameters
    double inflation_radius_ = 1.0; // 1.0 meter inflation radius
    int max_cost_ = 100;            // Max cost for lethal obstacles

    // Helper functions
    bool worldToGrid(double wx, double wy, int &gx, int &gy) const;
    void markObstacle(std::vector<std::vector<int>> &grid, int gx, int gy);
    void inflateObstacles(std::vector<std::vector<int>> &grid);
};

}  

#endif  