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

#ifndef SHISEN_OPENCV__NODE__CAMERA_CAPTURE_HPP_
#define SHISEN_OPENCV__NODE__CAMERA_CAPTURE_HPP_

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shisen_cpp/shisen_cpp.hpp>

#include <memory>
#include <string>

#include "../utility.hpp"

namespace shisen_opencv
{

class CameraCapture : public shisen_cpp::CameraNode
{
public:
  struct Options : public virtual shisen_cpp::CameraNode::Options
  {
    std::string camera_file_name;
    int capture_fps;

    Options()
    : camera_file_name("/dev/video0"),
      capture_fps(60)
    {
    }
  };

  explicit CameraCapture(rclcpp::Node::SharedPtr node, const Options & options = Options());
  ~CameraCapture();

  virtual void on_mat_captured(cv::Mat mat);
  virtual void on_camera_config(
    shisen_interfaces::msg::CameraConfig config,
    int width, int height);

  std::shared_ptr<cv::VideoCapture> get_video_capture() const;

private:
  rclcpp::TimerBase::SharedPtr capture_timer;

  std::shared_ptr<cv::VideoCapture> video_capture;
};

}  // namespace shisen_opencv

#endif  // SHISEN_OPENCV__NODE__CAMERA_CAPTURE_HPP_
