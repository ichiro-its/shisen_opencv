#ifndef SHISEN__CAMERA_HPP_
#define SHISEN__CAMERA_HPP_

#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <map>
#include <memory>
#include <string>

namespace shisen
{
  class Camera : public rclcpp::Node
  {
  public:

    Camera(std::string node_name);
    ~Camera();

    void open(std::string device_name);
    void open();

    void close();

  private:

    const std::map<std::string, int> property_names = {
      { "brightness", cv::CAP_PROP_BRIGHTNESS },
      { "contrast", cv::CAP_PROP_CONTRAST },
      { "saturation", cv::CAP_PROP_SATURATION },
      { "gain", cv::CAP_PROP_GAIN },
      { "exposure", cv::CAP_PROP_EXPOSURE },
      { "temperature", cv::CAP_PROP_TEMPERATURE }
    };

    cv::VideoCapture video_capture;

    std::shared_ptr<OnSetParametersCallbackHandle> on_set_parameter_handler;

    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>>
      compressed_image_publisher;

    std::shared_ptr<rclcpp::TimerBase> capture_timer;
  };
}

#endif