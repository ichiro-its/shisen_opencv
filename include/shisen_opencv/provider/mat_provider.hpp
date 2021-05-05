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

#ifndef SHISEN_OPENCV__PROVIDER__MAT_PROVIDER_HPP_
#define SHISEN_OPENCV__PROVIDER__MAT_PROVIDER_HPP_

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shisen_cpp/shisen_cpp.hpp>

#include <string>

#include "../utility.hpp"

namespace shisen_opencv
{

template<typename T>
class MatProvider;

using CompressedMatProvider = MatProvider<shisen_cpp::CompressedImage>;
using RawMatProvider = MatProvider<shisen_cpp::RawImage>;

template<typename T>
class MatProvider : public shisen_cpp::ImageProvider<T>
{
public:
  struct Options : public virtual shisen_cpp::ImageProvider<T>::Options
  {
  };

  inline explicit MatProvider(
    rclcpp::Node::SharedPtr node, const Options & options = Options());

  inline void set_mat_image(const MatImage & mat_image);
  inline void set_mat(cv::Mat mat);

  inline const MatImage & get_mat_image() const;
  inline cv::Mat get_mat() const;

private:
  MatImage current_mat_image;
};

template<typename T>
MatProvider<T>::MatProvider(rclcpp::Node::SharedPtr node, const MatProvider<T>::Options & options)
: shisen_cpp::ImageProvider<T>(node, options)
{
}

template<typename T>
void MatProvider<T>::set_mat_image(const MatImage & mat_image)
{
  current_mat_image = mat_image;
  this->set_image((T)get_mat_image());
}

template<typename T>
void MatProvider<T>::set_mat(cv::Mat mat)
{
  set_mat_image(mat);
}

template<typename T>
const MatImage & MatProvider<T>::get_mat_image() const
{
  return current_mat_image;
}

template<typename T>
cv::Mat MatProvider<T>::get_mat() const
{
  return (cv::Mat)get_mat_image();
}

}  // namespace shisen_opencv

#endif  // SHISEN_OPENCV__PROVIDER__MAT_PROVIDER_HPP_
