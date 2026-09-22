#include <memory>
#include <chrono>
#include <cmath>

#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  replan_sub_ = this->create_subscription<std_msgs::msg::Empty>(
    "/replan", 10, std::bind(&PlannerNode::replanCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));

  RCLCPP_INFO(this->get_logger(), "Planner Node initialized.");
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
  current_map_ = map;
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planAndPublishPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr goal) {
  current_goal_ = goal;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  RCLCPP_INFO(this->get_logger(), "Received new goal point: (%.2f, %.2f)", goal->point.x, goal->point.y);
  planAndPublishPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom) {
  current_odom_ = odom;
}

void PlannerNode::replanCallback(const std_msgs::msg::Empty::SharedPtr /*msg*/) {
  RCLCPP_INFO(this->get_logger(), "Received replan request from recovery handler. Computing new path.");
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planAndPublishPath();
  }
}

bool PlannerNode::goalReached() const {
  if (!current_goal_ || !current_odom_) return false;

  double dx = current_goal_->point.x - current_odom_->pose.pose.position.x;
  double dy = current_goal_->point.y - current_odom_->pose.pose.position.y;
  return std::sqrt(dx * dx + dy * dy) < goal_tolerance_;
}

void PlannerNode::planAndPublishPath() {
  if (!current_map_ || !current_goal_ || !current_odom_) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan path: missing map, goal, or odom data.");
    return;
  }

  auto path_opt = planner_.planPath(
    *current_map_,
    current_odom_->pose.pose.position,
    current_goal_->point);

  if (path_opt) {
    path_opt->header.stamp = this->get_clock()->now();
    path_pub_->publish(*path_opt);
  }
}

void PlannerNode::timerCallback() {
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    if (goalReached()) {
      RCLCPP_INFO(this->get_logger(), "Goal reached successfully!");
      state_ = State::WAITING_FOR_GOAL;
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
