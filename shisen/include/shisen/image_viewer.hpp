#ifndef SHISEN__IMAGE_VIEWER_HPP_
#define SHISEN__IMAGE_VIEWER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <shisen_interfaces/msg/compressed_image.hpp>
#include <shisen_interfaces/msg/raw_image.hpp>

#include <memory>
#include <string>

namespace shisen
{
  class ImageViewer : public rclcpp::Node
  {
  public:

    ImageViewer(std::string node_name, std::string topic_name);

  private:

    std::shared_ptr<rclcpp::Subscription<shisen_interfaces::msg::RawImage>>
      raw_image_subscription;

    std::shared_ptr<rclcpp::Subscription<shisen_interfaces::msg::CompressedImage>>
      compressed_image_subscription;
  };
}

#endif