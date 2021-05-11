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

MatImage::MatImage(const shisen_cpp::Image & image)
{
  *this = image;
}

MatImage::MatImage(cv::Mat mat)
{
  *this = mat;
}

MatImage::operator shisen_cpp::Image() const
{
  shisen_cpp::Image image;

  // Continue even if the serialization is failed
  try {
    image.cols = mat.cols;
    image.rows = mat.rows;
    image.channels = mat.channels();
    image.quality = -1;

    // Copy the mat data to the raw image
    auto byte_size = mat.total() * mat.elemSize();
    image.data.assign(mat.data, mat.data + byte_size);
  } catch (...) {
  }

  return image;
}

MatImage::operator cv::Mat() const
{
  return mat;
}

const MatImage & MatImage::operator=(const shisen_cpp::Image & image)
{
  // Continue even if the deserialization is failed
  try {
    // Determine whether the image is compressed or not
    if (image.quality < 0) {
      // Determine the mat type from the channel count
      auto type = CV_8UC1;
      if (image.channels == 2) {
        type = CV_8UC2;
      } else if (image.channels == 3) {
        type = CV_8UC3;
      } else if (image.channels == 4) {
        type = CV_8UC4;
      }

      mat = cv::Mat(image.rows, image.cols, type);

      // Copy the mat data from the raw image
      memcpy(mat.data, image.data.data(), image.data.size());
    } else {
      // Decode the compressed image
      mat = cv::imdecode(image.data, cv::IMREAD_UNCHANGED);
    }
  } catch (...) {
  }

  return *this;
}

const MatImage & MatImage::operator=(cv::Mat mat)
{
  this->mat = mat;

  return *this;
}

shisen_cpp::Image MatImage::compress(int quality)
{
  shisen_cpp::Image image;

  // Continue even if the serialization is failed
  try {
    image.cols = mat.cols;
    image.rows = mat.rows;
    image.channels = mat.channels();
    image.quality = quality;

    // Encode as a JPEG image
    std::vector<unsigned char> bytes;
    cv::imencode(".jpg", mat, bytes, {cv::IMWRITE_JPEG_QUALITY, image.quality});

    image.data = bytes;
  } catch (...) {
  }

  return image;
}

}  // namespace shisen_opencv
