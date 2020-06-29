#ifndef SHISEN__COMPRESSED_VIEWER_HPP_
#define SHISEN__COMPRESSED_VIEWER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <shisen_interfaces/msg/compressed_image.hpp>

#include <memory>
#include <string>

namespace shisen
{
  class CompressedViewer : public rclcpp::Node
  {
  public:

    CompressedViewer(std::string node_name, std::string topic_name);

  private:

    std::shared_ptr<rclcpp::Subscription<shisen_interfaces::msg::CompressedImage>>
      compressed_image_subscription;
  };
}

#endif