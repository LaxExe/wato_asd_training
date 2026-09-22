#include "control_core.hpp"
#include <algorithm>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger) : logger_(logger) {}

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion &q) const {
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double ControlCore::computeDistance(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2) const {
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  return std::sqrt(dx * dx + dy * dy);
}

bool ControlCore::isGoalReached(
  const nav_msgs::msg::Path &path,
  const nav_msgs::msg::Odometry &odom) const
{
  if (path.poses.empty()) return false;
  const auto &robot_pos = odom.pose.pose.position;
  const auto &final_goal = path.poses.back().pose.position;
  return computeDistance(robot_pos, final_goal) < goal_tolerance_;
}

std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint(
  const nav_msgs::msg::Path &path,
  const geometry_msgs::msg::Point &robot_pos) const
{
  if (path.poses.empty()) return std::nullopt;

  for (const auto &pose_stamped : path.poses) {
    double dist = computeDistance(robot_pos, pose_stamped.pose.position);
    if (dist >= lookahead_distance_) {
      return pose_stamped;
    }
  }

  return path.poses.back();
}

geometry_msgs::msg::Twist ControlCore::computeVelocity(
  const nav_msgs::msg::Path &path,
  const nav_msgs::msg::Odometry &odom)
{
  geometry_msgs::msg::Twist cmd_vel;

  if (path.poses.empty() || isGoalReached(path, odom)) {
    return cmd_vel; // Return 0 velocity when path empty or goal reached
  }

  const auto &robot_pos = odom.pose.pose.position;
  auto target_opt = findLookaheadPoint(path, robot_pos);
  if (!target_opt) return cmd_vel;

  const auto &target_pos = target_opt->pose.position;

  // Transform target into local robot frame
  double robot_yaw = extractYaw(odom.pose.pose.orientation);
  double dx = target_pos.x - robot_pos.x;
  double dy = target_pos.y - robot_pos.y;

  double x_rel = dx * std::cos(robot_yaw) + dy * std::sin(robot_yaw);
  double y_rel = -dx * std::sin(robot_yaw) + dy * std::cos(robot_yaw);

  double heading_angle = std::atan2(y_rel, x_rel);

  // In-Place Rotation for Sharp / 180 Turns
  if (std::abs(heading_angle) > M_PI / 2.0) {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = (heading_angle > 0.0) ? max_angular_speed_ : -max_angular_speed_;
    return cmd_vel;
  }

  // Standard Pure Pursuit with increased linear speed (1.0 m/s) and responsive angular control
  double dist_to_target = computeDistance(robot_pos, target_pos);
  double Ld = std::max(dist_to_target, 0.1);

  double curvature = (2.0 * y_rel) / (Ld * Ld);

  cmd_vel.linear.x = linear_speed_;
  cmd_vel.angular.z = std::clamp(linear_speed_ * curvature, -max_angular_speed_, max_angular_speed_);

  return cmd_vel;
}

}
