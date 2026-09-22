#include "planner_core.hpp"
#include <algorithm>
#include <cmath>
#include <queue>

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger) : logger_(logger) {}

bool PlannerCore::worldToGrid(const nav_msgs::msg::OccupancyGrid &map, double wx, double wy, int &gx, int &gy) const {
  double ox = map.info.origin.position.x;
  double oy = map.info.origin.position.y;
  double res = map.info.resolution;

  if (wx < ox || wy < oy) return false;

  gx = static_cast<int>((wx - ox) / res);
  gy = static_cast<int>((wy - oy) / res);

  return (gx >= 0 && gx < static_cast<int>(map.info.width) &&
          gy >= 0 && gy < static_cast<int>(map.info.height));
}

void PlannerCore::gridToWorld(const nav_msgs::msg::OccupancyGrid &map, int gx, int gy, double &wx, double &wy) const {
  double ox = map.info.origin.position.x;
  double oy = map.info.origin.position.y;
  double res = map.info.resolution;

  wx = ox + (gx + 0.5) * res;
  wy = oy + (gy + 0.5) * res;
}

double PlannerCore::heuristic(int x1, int y1, int x2, int y2) const {
  // True Euclidean distance heuristic for shortest geometric path
  double dx = x1 - x2;
  double dy = y1 - y2;
  return std::sqrt(dx * dx + dy * dy);
}

std::pair<int, int> PlannerCore::findClosestValidCell(
  const nav_msgs::msg::OccupancyGrid &map,
  int goal_x, int goal_y) const
{
  int width = map.info.width;
  int height = map.info.height;

  std::queue<std::pair<int, int>> q;
  std::vector<std::vector<bool>> visited(width, std::vector<bool>(height, false));

  q.push({goal_x, goal_y});
  visited[goal_x][goal_y] = true;

  const std::vector<std::pair<int, int>> dirs = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1},
    {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
  };

  while (!q.empty()) {
    auto [cx, cy] = q.front();
    q.pop();

    int cost = map.data[cy * width + cx];
    if (cost >= 0 && cost < obstacle_threshold_) {
      return {cx, cy};
    }

    for (const auto &d : dirs) {
      int nx = cx + d.first;
      int ny = cy + d.second;
      if (nx >= 0 && nx < width && ny >= 0 && ny < height && !visited[nx][ny]) {
        visited[nx][ny] = true;
        q.push({nx, ny});
      }
    }
  }

  return {goal_x, goal_y};
}

std::optional<nav_msgs::msg::Path> PlannerCore::planPath(
  const nav_msgs::msg::OccupancyGrid &map,
  const geometry_msgs::msg::Point &start_point,
  const geometry_msgs::msg::Point &goal_point)
{
  int start_x, start_y, raw_goal_x, raw_goal_y;
  if (!worldToGrid(map, start_point.x, start_point.y, start_x, start_y) ||
      !worldToGrid(map, goal_point.x, goal_point.y, raw_goal_x, raw_goal_y)) {
    RCLCPP_WARN(logger_, "Start or Goal point is outside the map boundaries!");
    return std::nullopt;
  }

  int width = map.info.width;
  int height = map.info.height;

  int goal_cost = map.data[raw_goal_y * width + raw_goal_x];
  int goal_x = raw_goal_x;
  int goal_y = raw_goal_y;

  if (goal_cost < 0 || goal_cost >= obstacle_threshold_) {
    auto [adj_x, adj_y] = findClosestValidCell(map, raw_goal_x, raw_goal_y);
    goal_x = adj_x;
    goal_y = adj_y;
    RCLCPP_INFO(logger_, "Target goal is unexplored/invalid (%d). Adjusting target goal to closest known cell (%d, %d).", goal_cost, goal_x, goal_y);
  }

  // 8-connected grid directions (cardinal + diagonal)
  const std::vector<std::pair<int, int>> directions = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1},       // Cardinal moves (cost = 1.0)
    {1, 1}, {1, -1}, {-1, 1}, {-1, -1}       // Diagonal moves (cost = 1.414)
  };

  std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_set;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;

  CellIndex start_cell{start_x, start_y};
  g_score[start_cell] = 0.0;
  open_set.push({start_cell, heuristic(start_x, start_y, goal_x, goal_y)});

  bool path_found = false;
  CellIndex final_cell;

  while (!open_set.empty()) {
    AStarNode current = open_set.top();
    open_set.pop();

    CellIndex curr_idx = current.index;

    if (std::abs(curr_idx.x - goal_x) <= 1 && std::abs(curr_idx.y - goal_y) <= 1) {
      path_found = true;
      final_cell = curr_idx;
      break;
    }

    double current_g = g_score[curr_idx];

    for (const auto &dir : directions) {
      int nx = curr_idx.x + dir.first;
      int ny = curr_idx.y + dir.second;

      if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;

      int map_cost = map.data[ny * width + nx];
      if (map_cost >= obstacle_threshold_) continue; // Skip lethal obstacles

      int effective_cost = (map_cost < 0) ? 5 : map_cost;

      // Base step cost: 1.0 for straight, 1.414 (sqrt(2)) for diagonal
      double base_step = (dir.first != 0 && dir.second != 0) ? M_SQRT2 : 1.0;

      // Small costmap obstacle weighting to stay safely off wall edges
      double step_cost = base_step + (effective_cost / 50.0);
      double tentative_g = current_g + step_cost;

      CellIndex neighbor{nx, ny};

      if (g_score.find(neighbor) == g_score.end() || tentative_g < g_score[neighbor]) {
        came_from[neighbor] = curr_idx;
        g_score[neighbor] = tentative_g;
        double f_val = tentative_g + heuristic(nx, ny, goal_x, goal_y);
        open_set.push({neighbor, f_val});
      }
    }
  }

  if (!path_found) {
    RCLCPP_WARN(logger_, "A* failed to find a valid path to the goal.");
    return std::nullopt;
  }

  nav_msgs::msg::Path path;
  path.header.frame_id = map.header.frame_id;

  std::vector<CellIndex> path_cells;
  CellIndex curr = final_cell;
  while (came_from.find(curr) != came_from.end()) {
    path_cells.push_back(curr);
    curr = came_from[curr];
  }
  path_cells.push_back(start_cell);
  std::reverse(path_cells.begin(), path_cells.end());

  for (const auto &cell : path_cells) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = map.header.frame_id;
    gridToWorld(map, cell.x, cell.y, pose.pose.position.x, pose.pose.position.y);
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  return path;
}

}
