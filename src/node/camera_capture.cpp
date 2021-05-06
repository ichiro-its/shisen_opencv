// Copyright (c) 2020-2021 ICHIRO ITS
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

#include <rclcpp/rclcpp.hpp>
#include <shisen_opencv/node/camera_capture.hpp>

#include <memory>

namespace shisen_opencv
{

using namespace std::chrono_literals;

CameraCapture::CameraCapture(rclcpp::Node::SharedPtr node, const CameraCapture::Options & options)
: node(node),
  video_capture(std::make_shared<cv::VideoCapture>())
{
  if (!video_capture->open(options.camera_file_name)) {
    throw std::runtime_error("unable to open camera on `" + options.camera_file_name + "`");
  }

  RCLCPP_INFO_STREAM(
    get_node()->get_logger(),
    "Camera capture opened on `" << options.camera_file_name << "`!");
}

CameraCapture::~CameraCapture()
{
  if (video_capture->isOpened()) {
    video_capture->release();
    RCLCPP_INFO(get_node()->get_logger(), "Camera capture closed!");
  } else {
    RCLCPP_WARN(get_node()->get_logger(), "Camera capture had not been opened!");
  }
}

rclcpp::Node::SharedPtr CameraCapture::get_node() const
{
  return node;
}

std::shared_ptr<cv::VideoCapture> CameraCapture::get_video_capture() const
{
  return video_capture;
}

}  // namespace shisen_opencv
