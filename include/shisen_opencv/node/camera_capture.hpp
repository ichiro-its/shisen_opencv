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

#include <memory>

#include "../utility.hpp"

namespace shisen_opencv
{

class CameraCapture
{
public:
  struct Options : public virtual CameraCaptureOptions
  {
  };

  explicit CameraCapture(rclcpp::Node::SharedPtr node, const Options & options = Options());
  ~CameraCapture();

  virtual void on_mat_captured(cv::Mat mat);

  rclcpp::Node::SharedPtr get_node() const;

  std::shared_ptr<cv::VideoCapture> get_video_capture() const;

private:
  rclcpp::Node::SharedPtr node;

  std::shared_ptr<rclcpp::TimerBase> capture_timer;

  std::shared_ptr<cv::VideoCapture> video_capture;
};

}  // namespace shisen_opencv

#endif  // SHISEN_OPENCV__NODE__CAMERA_CAPTURE_HPP_
