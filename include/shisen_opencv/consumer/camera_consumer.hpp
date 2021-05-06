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

#ifndef SHISEN_OPENCV__CONSUMER__CAMERA_CONSUMER_HPP_
#define SHISEN_OPENCV__CONSUMER__CAMERA_CONSUMER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <shisen_cpp/shisen_cpp.hpp>

#include "./mat_consumer.hpp"

namespace shisen_opencv
{

class CameraConsumer : public MatConsumer, public shisen_cpp::CaptureSettingConsumer
{
public:
  struct Options
    : public virtual MatConsumer::Options,
    public virtual shisen_cpp::CaptureSettingConsumer::Options
  {
  };

  inline explicit CameraConsumer(rclcpp::Node::SharedPtr node, const Options & options = Options());
};

CameraConsumer::CameraConsumer(
  rclcpp::Node::SharedPtr node, const CameraConsumer::Options & options)
: MatConsumer(node, options),
  CaptureSettingConsumer(node, options)
{
}

}  // namespace shisen_opencv

#endif  // SHISEN_OPENCV__CONSUMER__CAMERA_CONSUMER_HPP_
