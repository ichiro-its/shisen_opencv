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

#ifndef SHISEN_OPENCV__NODE__CAMERA_HPP_
#define SHISEN_OPENCV__NODE__CAMERA_HPP_

#include <rclcpp/rclcpp.hpp>

#include "../provider.hpp"
#include "./camera_capture.hpp"

namespace shisen_opencv
{

class Camera : public CameraCapture, public CameraProvider
{
public:
  struct Options : public virtual CameraCapture::Options, public virtual CameraProvider::Options
  {
  };

  explicit Camera(rclcpp::Node::SharedPtr node, const Options & options = Options());

  void on_mat_captured(cv::Mat mat) override;

  shisen_cpp::CaptureSetting on_configure_capture_setting(
    const shisen_cpp::CaptureSetting & capture_setting) override;
};

}  // namespace shisen_opencv

#endif  // SHISEN_OPENCV__NODE__CAMERA_HPP_
