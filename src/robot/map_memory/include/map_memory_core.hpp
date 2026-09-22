#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include <vector>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    void integrateCostmap(const nav_msgs::msg::OccupancyGrid &costmap, const nav_msgs::msg::Odometry &odom);
    nav_msgs::msg::OccupancyGrid getGlobalMap() const;

  private:
    rclcpp::Logger logger_;

    // Global Map Parameters
    double resolution_ = 0.1;
    int width_ = 600;           // 60m width
    int height_ = 600;          // 60m height
    double origin_x_ = -30.0;   // Centered world (0,0) at grid cell (300, 300)
    double origin_y_ = -30.0;

    std::vector<std::vector<int8_t>> global_grid_;

    // Helper functions
    double extractYaw(const geometry_msgs::msg::Quaternion &q) const;
};

}  

#endif  
