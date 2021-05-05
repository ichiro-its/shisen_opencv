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

#ifndef SHISEN_OPENCV__CONSUMER__COMBINED_MAT_CONSUMER_HPP_
#define SHISEN_OPENCV__CONSUMER__COMBINED_MAT_CONSUMER_HPP_

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shisen_cpp/shisen_cpp.hpp>

#include <memory>
#include <string>

#include "./member_mat_consumer.hpp"

namespace shisen_opencv
{

class CombinedMatConsumer
{
public:
  struct Options
    : public virtual CombinedMatOptions,
    public virtual MemberRawMatConsumer::Options,
    public virtual MemberCompressedMatConsumer::Options
  {
  };

  inline explicit CombinedMatConsumer(
    rclcpp::Node::SharedPtr node, const Options & options = Options());

  inline virtual void on_mat_changed(cv::Mat mat);

  inline void enable_raw(const bool & enable);
  inline void enable_compressed(const bool & enable);

  inline rclcpp::Node::SharedPtr get_node() const;

  inline bool is_raw_enabled() const;
  inline bool is_compressed_enabled() const;

  inline const MatImage & get_mat_image() const;
  inline cv::Mat get_mat() const;

private:
  rclcpp::Node::SharedPtr node;
  Options options;

  std::shared_ptr<MemberRawMatConsumer> member_raw_mat_provider;
  std::shared_ptr<MemberCompressedMatConsumer> member_compressed_mat_provider;

  MatImage current_mat_image;
};

CombinedMatConsumer::CombinedMatConsumer(
  rclcpp::Node::SharedPtr node, const CombinedMatConsumer::Options & options)
: node(node),
  options(options)
{
  this->enable_raw(options.enable_raw);
  this->enable_compressed(options.enable_compressed);
}

void CombinedMatConsumer::on_mat_changed(cv::Mat /*mat*/)
{
}

void CombinedMatConsumer::enable_raw(const bool & enable)
{
  if (enable) {
    member_raw_mat_provider = std::make_shared<MemberRawMatConsumer>(node, options);

    // Set on mat changed callback
    member_raw_mat_provider->set_on_mat_changed(
      [this](cv::Mat mat) {
        on_mat_changed(mat);
      });
  } else {
    member_raw_mat_provider = nullptr;
  }
}

void CombinedMatConsumer::enable_compressed(const bool & enable)
{
  if (enable) {
    member_compressed_mat_provider = std::make_shared<MemberCompressedMatConsumer>(node, options);

    // Set on mat changed callback
    member_compressed_mat_provider->set_on_mat_changed(
      [this](cv::Mat mat) {
        on_mat_changed(mat);
      });
  } else {
    member_compressed_mat_provider = nullptr;
  }
}

rclcpp::Node::SharedPtr CombinedMatConsumer::get_node() const
{
  return node;
}

bool CombinedMatConsumer::is_raw_enabled() const
{
  return member_raw_mat_provider != nullptr;
}

bool CombinedMatConsumer::is_compressed_enabled() const
{
  return member_compressed_mat_provider != nullptr;
}

const MatImage & CombinedMatConsumer::get_mat_image() const
{
  return current_mat_image;
}

cv::Mat CombinedMatConsumer::get_mat() const
{
  return (cv::Mat)get_mat_image();
}

}  // namespace shisen_opencv

#endif  // SHISEN_OPENCV__CONSUMER__COMBINED_MAT_CONSUMER_HPP_
