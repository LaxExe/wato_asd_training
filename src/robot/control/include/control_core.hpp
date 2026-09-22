#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include <optional>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace robot
{

class ControlCore {
  public:
    explicit ControlCore(const rclcpp::Logger& logger);

    geometry_msgs::msg::Twist computeVelocity(
      const nav_msgs::msg::Path &path,
      const nav_msgs::msg::Odometry &odom);

    bool isGoalReached(
      const nav_msgs::msg::Path &path,
      const nav_msgs::msg::Odometry &odom) const;

  private:
    rclcpp::Logger logger_;

    // Parameters
    double lookahead_distance_ = 0.8;  // 0.8 meter lookahead for agile tracking
    double goal_tolerance_ = 0.4;      // Goal stop distance tolerance (0.4m)
    double linear_speed_ = 1.0;         // Faster speed (1.0 m/s)
    double max_angular_speed_ = 2.0;   // Higher angular speed limit (2.0 rad/s)

    // Helpers
    double extractYaw(const geometry_msgs::msg::Quaternion &q) const;
    double computeDistance(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2) const;
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint(
      const nav_msgs::msg::Path &path,
      const geometry_msgs::msg::Point &robot_pos) const;
};

} 

#endif 
