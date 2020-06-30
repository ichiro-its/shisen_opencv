#include "shisen/image_viewer.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

using namespace shisen;

ImageViewer::ImageViewer(std::string node_name, std::string topic_name)
  : rclcpp::Node(node_name)
{
  raw_image_subscription = this->create_subscription<shisen_interfaces::msg::RawImage>(
    topic_name, 10,
    [this, topic_name](std::unique_ptr<shisen_interfaces::msg::RawImage> message) {
      cv::Mat received_frame(cv::Size(message->cols, message->rows),
        message->type, message->data.data());

      if (!received_frame.empty()) {
        cv::imshow(topic_name, received_frame);
        cv::waitKey(1);
        RCLCPP_DEBUG_ONCE(this->get_logger(), "once, received raw image and display it");
      }
      else {
        RCLCPP_WARN_ONCE(this->get_logger(), "once, received empty raw image");
      }
    }
  );

  RCLCPP_INFO_STREAM(get_logger(), "subscribe raw image on "
    << raw_image_subscription->get_topic_name());

  compressed_image_subscription = this->create_subscription<shisen_interfaces::msg::CompressedImage>(
    topic_name, 10,
    [this, topic_name](std::unique_ptr<shisen_interfaces::msg::CompressedImage> message) {
      auto received_frame = cv::imdecode(message->data, cv::IMREAD_UNCHANGED);
      if (!received_frame.empty()) {
        cv::imshow(topic_name, received_frame);
        cv::waitKey(1);
        RCLCPP_DEBUG_ONCE(this->get_logger(), "once, received compressed image and display it");
      }
      else {
        RCLCPP_WARN_ONCE(this->get_logger(), "once, received empty compressed image");
      }
    }
  );

  RCLCPP_INFO_STREAM(get_logger(), "subscribe compressed image on "
    << compressed_image_subscription->get_topic_name());
}