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

#include <shisen_opencv/provider/camera_config.hpp>

namespace shisen_opencv
{

CameraConfig::CameraConfig(
  rclcpp::Node::SharedPtr node, const CameraConfig::Options & options)
: shisen_cpp::CameraConfigProvider(node, options)
{
  field_of_view = options.field_of_view;
}

CameraConfig::~CameraConfig()
{
}

void CameraConfig::set_config(shisen_interfaces::msg::CameraConfig & config)
{
  float diagonal = pow(config.width * config.width + config.height * config.height, 0.5);
  float depth = (diagonal / 2) / keisan::make_degree(field_of_view / 2).tan();

  float view_h_angle =
    2 * keisan::signed_arctan(static_cast<float>(config.width / 2), depth).degree();
  float view_v_angle =
    2 * keisan::signed_arctan(static_cast<float>(config.height / 2), depth).degree();

  config.v_angle = view_v_angle;
  config.h_angle = view_h_angle;

  set_camera_config(config);  
}

}  // namespace shisen_opencv
