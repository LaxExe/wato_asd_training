#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/empty.hpp"

#include "control_core.hpp"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:
    enum class RobotState {
      NORMAL,
      REVERSING
    };

    void pathCallback(const nav_msgs::msg::Path::SharedPtr path);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
    void controlLoopTimer();

    bool isRobotInCostmapObstacle() const;

    robot::ControlCore control_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr replan_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Path::SharedPtr current_path_;
    nav_msgs::msg::Odometry::SharedPtr current_odom_;
    nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;

    // Recovery & Stuck Detection
    RobotState state_ = RobotState::NORMAL;
    rclcpp::Time last_moved_time_;
    rclcpp::Time reverse_start_time_;
    geometry_msgs::msg::Point last_odom_pos_;
    bool first_odom_ = true;
};

#endif
