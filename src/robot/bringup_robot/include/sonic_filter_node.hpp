#ifndef SONIC_FILTER_NODE_HPP_
#define SONIC_FILTER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>

class SonicFilterNode : public rclcpp::Node {
  public:
    SonicFilterNode();

  private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    void applySonicAesthetic(cv::Mat& background);

    cv::Mat sonic_overlay_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
};

#endif
