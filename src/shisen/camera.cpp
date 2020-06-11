#include "shisen/camera.hpp"

#include <opencv2/imgcodecs.hpp>

#include <chrono>

using namespace shisen;
using namespace std::chrono_literals;

Camera::Camera(std::string node_name) : rclcpp::Node(node_name)
{
  compressed_image_publisher = this->create_publisher<sensor_msgs::msg::CompressedImage>(
    node_name + "/compressed_image", 10
  );

  RCLCPP_INFO_STREAM(get_logger(), "publish compressed image on "
    << compressed_image_publisher->get_topic_name());

  capture_timer = this->create_wall_timer(30ms, [this]() {
    if (!video_capture.isOpened())
      return;

    cv::Mat captured_frame;
    video_capture.read(captured_frame);

    if (!captured_frame.empty()) {
      sensor_msgs::msg::CompressedImage message;

      message.format = "png";

      std::vector<unsigned char> bytes;
      cv::imencode(".png", captured_frame, bytes);
      message.data = bytes;

      compressed_image_publisher->publish(message);

      RCLCPP_DEBUG(get_logger(), "captured image and publish it");
    }
    else {
      RCLCPP_WARN(get_logger(), "captured empty image");
    }
  });
}

void Camera::open(std::string device_name)
{
  if (video_capture.open(device_name))
    RCLCPP_INFO_STREAM(get_logger(), "opened video capture on " << device_name);
  else
    RCLCPP_ERROR_STREAM(get_logger(), "unable to open video capture on " << device_name);
}

void Camera::open()
{
  open("/dev/video0");
}
