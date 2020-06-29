#include "shisen/raw_viewer.hpp"

#include <opencv2/highgui.hpp>

using namespace shisen;

RawViewer::RawViewer(std::string node_name, std::string topic_name)
  : rclcpp::Node(node_name)
{
  raw_image_subscription = this->create_subscription<shisen_interfaces::msg::RawImage>(
    topic_name, 10,
    [this, topic_name](std::unique_ptr<shisen_interfaces::msg::RawImage> message) {
      RCLCPP_DEBUG_STREAM_ONCE(this->get_logger(), "once, image is " << message->type
        << " size of " << message->cols << " x " << message->rows
        << " (" << message->data.size() << ")");

      cv::Mat received_frame(cv::Size(message->cols, message->rows),
        message->type, message->data.data());

      if (!received_frame.empty()) {
        cv::imshow(topic_name, received_frame);
        cv::waitKey(1);
        RCLCPP_DEBUG_ONCE(this->get_logger(), "once, received image and display it");
      }
      else {
        RCLCPP_WARN_ONCE(this->get_logger(), "once, received empty image");
      }
    }
  );

  RCLCPP_INFO_STREAM(get_logger(), "subscribe raw image on "
    << raw_image_subscription->get_topic_name());
}