#include <memory>
#include <chrono>
#include <cmath>

#include "control_node.hpp"

ControlNode::ControlNode() : Node("control"), control_(robot::ControlCore(this->get_logger())) {
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&ControlNode::mapCallback, this, std::placeholders::_1));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  replan_pub_ = this->create_publisher<std_msgs::msg::Empty>("/replan", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&ControlNode::controlLoopTimer, this));

  last_moved_time_ = this->get_clock()->now();
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr path) {
  current_path_ = path;
  last_moved_time_ = this->get_clock()->now();
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom) {
  current_odom_ = odom;

  if (first_odom_) {
    last_odom_pos_ = odom->pose.pose.position;
    last_moved_time_ = this->get_clock()->now();
    first_odom_ = false;
    return;
  }

  double dx = odom->pose.pose.position.x - last_odom_pos_.x;
  double dy = odom->pose.pose.position.y - last_odom_pos_.y;
  double dist = std::sqrt(dx * dx + dy * dy);

  if (dist > 0.05) {
    last_moved_time_ = this->get_clock()->now();
    last_odom_pos_ = odom->pose.pose.position;
  }
}

void ControlNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
  current_map_ = map;
}

bool ControlNode::isRobotInCostmapObstacle() const {
  if (!current_map_ || !current_odom_) return false;

  double rx = current_odom_->pose.pose.position.x;
  double ry = current_odom_->pose.pose.position.y;

  double ox = current_map_->info.origin.position.x;
  double oy = current_map_->info.origin.position.y;
  double res = current_map_->info.resolution;

  if (rx < ox || ry < oy) return false;

  int gx = static_cast<int>((rx - ox) / res);
  int gy = static_cast<int>((ry - oy) / res);

  int width = current_map_->info.width;
  int height = current_map_->info.height;

  if (gx >= 0 && gx < width && gy >= 0 && gy < height) {
    int cost = current_map_->data[gy * width + gx];
    return cost > 0;
  }

  return false;
}

void ControlNode::controlLoopTimer() {
  if (!current_odom_) return;

  rclcpp::Time now = this->get_clock()->now();

  if (!current_path_ || current_path_->poses.empty()) {
    geometry_msgs::msg::Twist stop_cmd;
    cmd_vel_pub_->publish(stop_cmd);
    return;
  }

  if (control_.isGoalReached(*current_path_, *current_odom_)) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Goal reached, stopping robot.");
    current_path_.reset();
    last_moved_time_ = now;

    geometry_msgs::msg::Twist stop_cmd;
    cmd_vel_pub_->publish(stop_cmd);
    return;
  }

  if (state_ == RobotState::REVERSING) {
    double reverse_elapsed = (now - reverse_start_time_).seconds();
    bool inside_obstacle = isRobotInCostmapObstacle();

    if ((inside_obstacle || reverse_elapsed < 1.5) && reverse_elapsed < 5.0) {
      geometry_msgs::msg::Twist reverse_cmd;
      reverse_cmd.linear.x = -0.35;
      reverse_cmd.angular.z = 0.0;
      cmd_vel_pub_->publish(reverse_cmd);
      return;
    } else {
      RCLCPP_INFO(this->get_logger(), "Cleared obstacle after reverse. Requesting path update.");
      state_ = RobotState::NORMAL;
      last_moved_time_ = now;

      current_path_.reset();
      std_msgs::msg::Empty replan_msg;
      replan_pub_->publish(replan_msg);
      return;
    }
  }

  if ((now - last_moved_time_).seconds() > 2.0) {
    RCLCPP_WARN(this->get_logger(), "Robot stuck, starting recovery reverse.");
    state_ = RobotState::REVERSING;
    reverse_start_time_ = now;
    return;
  }

  auto cmd_vel = control_.computeVelocity(*current_path_, *current_odom_);
  cmd_vel_pub_->publish(cmd_vel);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}

