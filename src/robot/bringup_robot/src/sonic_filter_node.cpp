#include "sonic_filter_node.hpp"

SonicFilterNode::SonicFilterNode() : Node("sonic_filter_node") {
  std::string overlay_path = "/home/bolty/ament_ws/src/robot/bringup_robot/Sonic-Movie-PNG.png";
  sonic_overlay_ = cv::imread(overlay_path, cv::IMREAD_UNCHANGED);

  if (sonic_overlay_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load overlay from: %s", overlay_path.c_str());
  } else {
    if (sonic_overlay_.cols > 150 || sonic_overlay_.rows > 150) {
      double scale = 150.0 / std::max(sonic_overlay_.cols, sonic_overlay_.rows);
      cv::resize(sonic_overlay_, sonic_overlay_, cv::Size(), scale, scale, cv::INTER_AREA);
    }
  }

  image_transport::ImageTransport it(this->shared_from_this());

  image_sub_ = it.subscribe("/camera/image_raw", 10,
    std::bind(&SonicFilterNode::imageCallback, this, std::placeholders::_1));

  image_pub_ = it.advertise("/camera/sonic_image", 10);
}

void SonicFilterNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
  try {
    cv::Mat cv_image = cv_bridge::toCvCopy(msg, "bgr8")->image;

    applySonicAesthetic(cv_image);

    sensor_msgs::msg::Image::SharedPtr modified_msg =
      cv_bridge::CvImage(msg->header, "bgr8", cv_image).toImageMsg();

    image_pub_.publish(modified_msg);

  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }
}

void SonicFilterNode::applySonicAesthetic(cv::Mat& background) {
  if (sonic_overlay_.empty()) return;

  int y_start = 20;
  int x_start = 20;

  if (y_start + sonic_overlay_.rows > background.rows ||
      x_start + sonic_overlay_.cols > background.cols) {
    return;
  }

  for (int y = 0; y < sonic_overlay_.rows; y++) {
    for (int x = 0; x < sonic_overlay_.cols; x++) {
      cv::Vec4b overlay_pixel = sonic_overlay_.at<cv::Vec4b>(y, x);
      if (overlay_pixel[3] > 0) {
        cv::Vec3b& bg_pixel = background.at<cv::Vec3b>(y + y_start, x + x_start);
        float alpha = static_cast<float>(overlay_pixel[3]) / 255.0f;
        bg_pixel[0] = static_cast<uchar>((1 - alpha) * bg_pixel[0] + alpha * overlay_pixel[0]);
        bg_pixel[1] = static_cast<uchar>((1 - alpha) * bg_pixel[1] + alpha * overlay_pixel[1]);
        bg_pixel[2] = static_cast<uchar>((1 - alpha) * bg_pixel[2] + alpha * overlay_pixel[2]);
      }
    }
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SonicFilterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

