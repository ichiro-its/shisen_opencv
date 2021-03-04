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

#ifndef SHISEN__CAMERA_HPP_
#define SHISEN__CAMERA_HPP_

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

class Camera : public rclcpp::Node
{
public:
  explicit Camera(std::string node_name);
  ~Camera();

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

}  // namespace shisen

#endif  // SHISEN__CAMERA_HPP_
