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

#include <opencv2/imgcodecs.hpp>
#include <shisen_opencv/utility/mat_image.hpp>

#include <vector>

namespace shisen_opencv
{

MatImage::MatImage()
{
}

MatImage::MatImage(const shisen_cpp::CompressedImage & compressed_image)
{
  *this = compressed_image;
}

MatImage::MatImage(const shisen_cpp::RawImage & raw_image)
{
  *this = raw_image;
}

MatImage::MatImage(cv::Mat mat)
{
  *this = mat;
}

MatImage::operator shisen_cpp::CompressedImage() const
{
  shisen_cpp::CompressedImage compressed_image;

  // Encode as a JPEG image with quality of 50
  std::vector<unsigned char> bytes;
  cv::imencode(".jpg", mat, bytes, {cv::IMWRITE_JPEG_QUALITY, 50});

  compressed_image.data = bytes;

  return compressed_image;
}

MatImage::operator shisen_cpp::RawImage() const
{
  shisen_cpp::RawImage raw_image;

  raw_image.cols = mat.cols;
  raw_image.rows = mat.rows;
  raw_image.channels = mat.channels();

  // Copy the mat data to the raw image
  auto byte_size = mat.total() * mat.elemSize();
  raw_image.data.assign(mat.data, mat.data + byte_size);

  return raw_image;
}

MatImage::operator cv::Mat() const
{
  return mat;
}

const MatImage & MatImage::operator=(const shisen_cpp::CompressedImage & compressed_image)
{
  mat = cv::imdecode(compressed_image.data, cv::IMREAD_UNCHANGED);

  return *this;
}

const MatImage & MatImage::operator=(const shisen_cpp::RawImage & raw_image)
{
  // Determine the mat type from the channel count
  auto type = CV_8UC1;
  if (raw_image.channels == 2) {
    type = CV_8UC2;
  } else if (raw_image.channels == 3) {
    type = CV_8UC3;
  } else if (raw_image.channels == 4) {
    type = CV_8UC4;
  }

  mat = cv::Mat(raw_image.rows, raw_image.cols, type);

  // Copy the mat data from the raw image
  memcpy(mat.data, raw_image.data.data(), raw_image.data.size());

  return *this;
}

const MatImage & MatImage::operator=(cv::Mat mat)
{
  this->mat = mat;

  return *this;
}

}  // namespace shisen_opencv
