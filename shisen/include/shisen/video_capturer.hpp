#ifndef SHISEN__VIDEO_CAPTURER_HPP_
#define SHISEN__VIDEO_CAPTURER_HPP_

#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shisen_interfaces/msg/compressed_image.hpp>
#include <shisen_interfaces/msg/property_event.hpp>
#include <shisen_interfaces/msg/raw_image.hpp>
#include <shisen_interfaces/srv/get_properties.hpp>
#include <shisen_interfaces/srv/set_properties.hpp>

#include <map>
#include <memory>
#include <string>

namespace shisen
{
  class VideoCapturer : public rclcpp::Node
  {
  public:

    VideoCapturer(std::string node_name);
    ~VideoCapturer();

    bool open(std::string file_name);
    bool close();

  private:

    static const std::map<std::string, int> property_ids;

    cv::VideoCapture video_capture;

    std::map<std::string, double> property_map;

    std::shared_ptr<rclcpp::Publisher<shisen_interfaces::msg::RawImage>>
      raw_image_publisher;

    std::shared_ptr<rclcpp::Publisher<shisen_interfaces::msg::CompressedImage>>
      compressed_image_publisher;

    std::shared_ptr<rclcpp::Publisher<shisen_interfaces::msg::PropertyEvent>>
      property_event_publisher;

    std::shared_ptr<rclcpp::Service<shisen_interfaces::srv::GetProperties>>
      get_properties_service;

    std::shared_ptr<rclcpp::Service<shisen_interfaces::srv::SetProperties>>
      set_properties_service;

    std::shared_ptr<rclcpp::TimerBase> capture_timer;
  };
}

#endif