#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <optional>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace robot
{

struct CellIndex {
  int x;
  int y;

  bool operator==(const CellIndex &other) const {
    return x == other.x && y == other.y;
  }
};

struct CellIndexHash {
  std::size_t operator()(const CellIndex &idx) const {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

struct AStarNode {
  CellIndex index;
  double f_score;

  bool operator>(const AStarNode &other) const {
    return f_score > other.f_score;
  }
};

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    std::optional<nav_msgs::msg::Path> planPath(
      const nav_msgs::msg::OccupancyGrid &map,
      const geometry_msgs::msg::Point &start_point,
      const geometry_msgs::msg::Point &goal_point);

  private:
    rclcpp::Logger logger_;

    int obstacle_threshold_ = 50; // Lethal obstacle threshold

    bool worldToGrid(const nav_msgs::msg::OccupancyGrid &map, double wx, double wy, int &gx, int &gy) const;
    void gridToWorld(const nav_msgs::msg::OccupancyGrid &map, int gx, int gy, double &wx, double &wy) const;
    double heuristic(int x1, int y1, int x2, int y2) const;

    // Nearest valid cell search for unexplored/invalid target goals
    std::pair<int, int> findClosestValidCell(
      const nav_msgs::msg::OccupancyGrid &map,
      int goal_x, int goal_y) const;
};

}  

#endif  
