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

#include <shisen_opencv/consumer/mat_consumer.hpp>

namespace shisen_opencv
{

MatConsumer::MatConsumer(rclcpp::Node::SharedPtr node, const MatConsumer::Options & options)
: shisen_cpp::ImageConsumer(node, options)
{
}

MatConsumer::~MatConsumer()
{
}

void MatConsumer::on_image_changed(const shisen_cpp::Image & image)
{
  // Call parent's overridden function
  shisen_cpp::ImageConsumer::on_image_changed(image);

  current_mat_image = image;

  // Call virtual callback
  on_mat_changed(get_mat());
}

void MatConsumer::on_mat_changed(cv::Mat /*mat*/)
{
}

cv::Mat MatConsumer::get_mat() const
{
  return (cv::Mat)current_mat_image;
}

}  // namespace shisen_opencv
