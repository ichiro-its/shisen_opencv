// Copyright 2020-2021 Ichiro ITS
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

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