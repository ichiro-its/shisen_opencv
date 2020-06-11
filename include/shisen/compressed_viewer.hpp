#ifndef SHISEN__COMPRESSED_VIEWER_HPP_
#define SHISEN__COMPRESSED_VIEWER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <memory>
#include <string>

namespace shisen
{
  class CompressedViewer : public rclcpp::Node
  {
  public:

    CompressedViewer(std::string node_name, std::string topic_name);

  private:

    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::CompressedImage>>
      compressed_image_subscription;
  };
}

#endif