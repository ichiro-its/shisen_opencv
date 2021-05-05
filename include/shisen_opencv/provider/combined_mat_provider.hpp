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

#ifndef SHISEN_OPENCV__PROVIDER__COMBINED_MAT_PROVIDER_HPP_
#define SHISEN_OPENCV__PROVIDER__COMBINED_MAT_PROVIDER_HPP_

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shisen_cpp/shisen_cpp.hpp>

#include <memory>
#include <string>

#include "./mat_provider.hpp"

namespace shisen_opencv
{

class CombinedMatProvider
{
public:
  inline explicit CombinedMatProvider(
    rclcpp::Node::SharedPtr node, const bool & enable_raw = true,
    const bool & enable_compressed = true, const std::string & prefix = shisen_cpp::CAMERA_PREFIX);

  inline void enable_raw(const bool & enable);
  inline void enable_compressed(const bool & enable);

  inline void set_mat_image(const MatImage & mat_image);
  inline void set_mat(cv::Mat mat);

  inline rclcpp::Node::SharedPtr get_node() const;

  inline bool is_raw_enabled() const;
  inline bool is_compressed_enabled() const;

  inline const MatImage & get_mat_image() const;
  inline cv::Mat get_mat() const;

private:
  rclcpp::Node::SharedPtr node;
  std::string prefix;

  std::shared_ptr<RawMatProvider> raw_mat_provider;
  std::shared_ptr<CompressedMatProvider> compressed_mat_provider;

  MatImage current_mat_image;
};

CombinedMatProvider::CombinedMatProvider(
  rclcpp::Node::SharedPtr node, const bool & enable_raw,
  const bool & enable_compressed, const std::string & prefix)
: node(node),
  prefix(prefix)
{
  this->enable_raw(enable_raw);
  this->enable_compressed(enable_compressed);
}

void CombinedMatProvider::enable_raw(const bool & enable)
{
  if (enable) {
    raw_mat_provider = std::make_shared<RawMatProvider>(node, prefix);
  } else {
    raw_mat_provider = nullptr;
  }
}

void CombinedMatProvider::enable_compressed(const bool & enable)
{
  if (enable) {
    compressed_mat_provider = std::make_shared<CompressedMatProvider>(node, prefix);
  } else {
    compressed_mat_provider = nullptr;
  }
}

void CombinedMatProvider::set_mat_image(const MatImage & mat_image)
{
  current_mat_image = mat_image;

  if (is_raw_enabled()) {
    raw_mat_provider->set_mat_image(get_mat_image());
  }

  if (is_compressed_enabled()) {
    compressed_mat_provider->set_mat_image(get_mat_image());
  }
}

void CombinedMatProvider::set_mat(cv::Mat mat)
{
  set_mat_image(MatImage(mat));
}

rclcpp::Node::SharedPtr CombinedMatProvider::get_node() const
{
  return node;
}

bool CombinedMatProvider::is_raw_enabled() const
{
  return raw_mat_provider != nullptr;
}

bool CombinedMatProvider::is_compressed_enabled() const
{
  return compressed_mat_provider != nullptr;
}

const MatImage & CombinedMatProvider::get_mat_image() const
{
  return current_mat_image;
}

cv::Mat CombinedMatProvider::get_mat() const
{
  return (cv::Mat)get_mat_image();
}

}  // namespace shisen_opencv

#endif  // SHISEN_OPENCV__PROVIDER__COMBINED_MAT_PROVIDER_HPP_
