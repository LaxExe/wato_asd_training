#include <memory>
#include <chrono>
#include <cmath>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  timer_ = this->create_wall_timer(
    std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMapTimer, this));

  RCLCPP_INFO(this->get_logger(), "Map Memory Node initialized.");
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap) {
  latest_costmap_ = costmap;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom) {
  latest_odom_ = odom;

  if (!latest_costmap_) return;

  double x = odom->pose.pose.position.x;
  double y = odom->pose.pose.position.y;

  double dist = std::sqrt(std::pow(x - last_x_, 2) + std::pow(y - last_y_, 2));

  // Integrate map if robot moved past threshold distance or on first observation
  if (!has_integrated_ || dist >= distance_threshold_) {
    map_memory_.integrateCostmap(*latest_costmap_, *odom);
    last_x_ = x;
    last_y_ = y;
    has_integrated_ = true;

    // Publish immediately on integration
    auto map_msg = map_memory_.getGlobalMap();
    map_msg.header.stamp = this->get_clock()->now();
    map_pub_->publish(map_msg);
  }
}

void MapMemoryNode::updateMapTimer() {
  if (has_integrated_) {
    auto map_msg = map_memory_.getGlobalMap();
    map_msg.header.stamp = this->get_clock()->now();
    map_pub_->publish(map_msg);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
