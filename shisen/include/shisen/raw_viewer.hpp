#ifndef SHISEN__RAW_VIEWER_HPP_
#define SHISEN__RAW_VIEWER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <shisen_interfaces/msg/raw_image.hpp>

#include <memory>
#include <string>

namespace shisen
{
  class RawViewer : public rclcpp::Node
  {
  public:

    RawViewer(std::string node_name, std::string topic_name);

  private:

    std::shared_ptr<rclcpp::Subscription<shisen_interfaces::msg::RawImage>>
      raw_image_subscription;
  };
}

#endif