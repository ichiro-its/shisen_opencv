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

#include <shisen_opencv/node/camera.hpp>

namespace shisen_opencv
{

Camera::Camera(rclcpp::Node::SharedPtr node, const Camera::Options & options)
: CameraCapture(node, options),
  CameraProvider(node, options)
{
}

void Camera::on_mat_captured(cv::Mat mat)
{
  // Call parent's overridden function
  CameraCapture::on_mat_captured(mat);

  set_mat(mat);
}

shisen_cpp::CaptureSetting Camera::on_configure_capture_setting(
  const shisen_cpp::CaptureSetting & capture_setting)
{
  // Call parent's overridden function
  auto new_capture_setting = CameraProvider::on_configure_capture_setting(capture_setting);

  auto video_capture = get_video_capture();

  // Set the capture setting to the camera
  {
    if (new_capture_setting.brightness.is_not_empty()) {
      video_capture->set(cv::CAP_PROP_BRIGHTNESS, new_capture_setting.brightness);
    }

    if (new_capture_setting.contrast.is_not_empty()) {
      video_capture->set(cv::CAP_PROP_CONTRAST, new_capture_setting.contrast);
    }

    if (new_capture_setting.saturation.is_not_empty()) {
      video_capture->set(cv::CAP_PROP_SATURATION, new_capture_setting.saturation);
    }

    if (new_capture_setting.temperature.is_not_empty()) {
      video_capture->set(cv::CAP_PROP_TEMPERATURE, new_capture_setting.temperature);
    }

    if (new_capture_setting.hue.is_not_empty()) {
      video_capture->set(cv::CAP_PROP_HUE, new_capture_setting.hue);
    }

    if (new_capture_setting.gain.is_not_empty()) {
      video_capture->set(cv::CAP_PROP_GAIN, new_capture_setting.gain);
    }
  }

  // Get new capture setting from the camera
  new_capture_setting.brightness = video_capture->get(cv::CAP_PROP_BRIGHTNESS);
  new_capture_setting.contrast = video_capture->get(cv::CAP_PROP_CONTRAST);
  new_capture_setting.saturation = video_capture->get(cv::CAP_PROP_SATURATION);
  new_capture_setting.temperature = video_capture->get(cv::CAP_PROP_TEMPERATURE);
  new_capture_setting.hue = video_capture->get(cv::CAP_PROP_HUE);
  new_capture_setting.gain = video_capture->get(cv::CAP_PROP_GAIN);

  return new_capture_setting;
}

}  // namespace shisen_opencv
