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

#ifndef SHISEN_OPENCV__CONSUMER__MEMBER_MAT_CONSUMER_HPP_
#define SHISEN_OPENCV__CONSUMER__MEMBER_MAT_CONSUMER_HPP_

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shisen_cpp/shisen_cpp.hpp>

#include <string>

#include "./mat_consumer.hpp"

namespace shisen_opencv
{

template<typename T>
class MemberMatConsumer;

using MemberCompressedMatConsumer = MemberMatConsumer<shisen_cpp::CompressedImage>;
using MemberRawMatConsumer = MemberMatConsumer<shisen_cpp::RawImage>;

template<typename T>
class MemberMatConsumer : public MatConsumer<T>
{
public:
  using MatCallback = std::function<void (cv::Mat)>;

  struct Options : public virtual MatConsumer<T>::Options
  {
  };

  inline explicit MemberMatConsumer(
    rclcpp::Node::SharedPtr node, const Options & options = Options());

  inline void on_mat_changed(cv::Mat mat) override;

  inline void set_on_mat_changed(const MatCallback & callback);

private:
  MatCallback on_mat_changed_callback;
};

template<typename T>
MemberMatConsumer<T>::MemberMatConsumer(
  rclcpp::Node::SharedPtr node, const MemberMatConsumer::Options & options)
: MatConsumer<T>(node, options)
{
}

template<typename T>
void MemberMatConsumer<T>::on_mat_changed(cv::Mat mat)
{
  // Call parent's overridden function
  MatConsumer<T>::on_mat_changed(mat);

  if (on_mat_changed_callback) {
    on_mat_changed_callback(mat);
  }
}

template<typename T>
void MemberMatConsumer<T>::set_on_mat_changed(const MatCallback & callback)
{
  on_mat_changed_callback = callback;
}

}  // namespace shisen_opencv

#endif  // SHISEN_OPENCV__CONSUMER__MEMBER_MAT_CONSUMER_HPP_
