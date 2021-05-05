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

#ifndef SHISEN_OPENCV__PROVIDER__CAMERA_PROVIDER_HPP_
#define SHISEN_OPENCV__PROVIDER__CAMERA_PROVIDER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <shisen_cpp/shisen_cpp.hpp>

#include <string>

#include "./combined_mat_provider.hpp"

namespace shisen_opencv
{

class CameraProvider : public CombinedMatProvider, public shisen_cpp::CaptureSettingProvider
{
public:
  inline explicit CameraProvider(
    rclcpp::Node::SharedPtr node, const bool & enable_raw = true,
    const bool & enable_compressed = true, const std::string & prefix = shisen_cpp::CAMERA_PREFIX);

  inline rclcpp::Node::SharedPtr get_node() const;

private:
  rclcpp::Node::SharedPtr node;
};

CameraProvider::CameraProvider(
  rclcpp::Node::SharedPtr node, const bool & enable_raw,
  const bool & enable_compressed, const std::string & prefix)
: CombinedMatProvider(node, enable_raw, enable_compressed, prefix),
  CaptureSettingProvider(node, prefix),
  node(node)
{
}

inline rclcpp::Node::SharedPtr CameraProvider::get_node() const
{
  return node;
}

}  // namespace shisen_opencv

#endif  // SHISEN_OPENCV__PROVIDER__CAMERA_PROVIDER_HPP_
