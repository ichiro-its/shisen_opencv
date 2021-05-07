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

#include <opencv2/highgui.hpp>

#include <shisen_opencv/node/viewer.hpp>

namespace shisen_opencv
{

Viewer::Viewer(rclcpp::Node::SharedPtr node, const Viewer::Options & options)
: CombinedMatConsumer(node, options)
{
}

void Viewer::on_mat_changed(cv::Mat mat)
{
  // Ensure the received mat is not empty
  if (!mat.empty()) {
    cv::imshow("viewer", mat);
    cv::waitKey(1);
  } else {
    RCLCPP_WARN_ONCE(get_node()->get_logger(), "Once, received an empty mat!");
  }
}

}  // namespace shisen_opencv
