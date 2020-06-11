#ifndef SHISEN__CAMERA_HPP_
#define SHISEN__CAMERA_HPP_

#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <memory>
#include <string>

namespace shisen
{
  class Camera : public rclcpp::Node
  {
  public:

    Camera(std::string node_name);

    void open(std::string device_name);
    void open();

  private:

    cv::VideoCapture video_capture;

    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>>
      compressed_image_publisher;

    std::shared_ptr<rclcpp::TimerBase> capture_timer;
  };
}

#endif