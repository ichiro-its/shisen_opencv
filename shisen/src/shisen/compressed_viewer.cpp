#include "shisen/compressed_viewer.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

using namespace shisen;

CompressedViewer::CompressedViewer(std::string node_name, std::string topic_name)
  : rclcpp::Node(node_name)
{
  compressed_image_subscription = this->create_subscription<shisen_interfaces::msg::CompressedImage>(
    topic_name, 10,
    [this, topic_name](std::unique_ptr<shisen_interfaces::msg::CompressedImage> message) {
      auto received_frame = cv::imdecode(message->data, cv::IMREAD_UNCHANGED);
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

  RCLCPP_INFO_STREAM(get_logger(), "subscribe compressed image on "
    << compressed_image_subscription->get_topic_name());
}