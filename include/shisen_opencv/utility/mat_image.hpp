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

#ifndef SHISEN_OPENCV__UTILITY__MAT_IMAGE_HPP_
#define SHISEN_OPENCV__UTILITY__MAT_IMAGE_HPP_

#include <shisen_cpp/shisen_cpp.hpp>
#include <opencv2/core.hpp>

namespace shisen_opencv
{

class MatImage
{
public:
  MatImage();
  explicit MatImage(const shisen_cpp::CompressedImage & compressed_image);
  explicit MatImage(const shisen_cpp::RawImage & raw_image);
  explicit MatImage(cv::Mat mat);

  operator shisen_cpp::CompressedImage() const;
  operator shisen_cpp::RawImage() const;
  operator cv::Mat() const;

  const MatImage & operator=(const shisen_cpp::CompressedImage & compressed_image);
  const MatImage & operator=(const shisen_cpp::RawImage & raw_image);
  const MatImage & operator=(cv::Mat mat);

// private:
  cv::Mat mat;
};

}  // namespace shisen_opencv

#endif  // SHISEN_OPENCV__UTILITY__MAT_IMAGE_HPP_
