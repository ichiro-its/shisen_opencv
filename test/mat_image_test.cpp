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

#include <gtest/gtest.h>
#include <opencv2/core.hpp>
#include <shisen_cpp/shisen_cpp.hpp>
#include <shisen_opencv/shisen_opencv.hpp>

#define ASSERT_MAT_IMAGE_SIZE_EQ(A, B) \
  { \
    cv::Mat _a = (cv::Mat)A; \
    cv::Mat _b = (cv::Mat)B; \
    ASSERT_EQ(_a.type(), _b.type()); \
    ASSERT_EQ(_a.cols, _b.cols); \
    ASSERT_EQ(_a.rows, _b.rows); \
    ASSERT_EQ(_a.channels(), _b.channels()); \
  }

#define ASSERT_MAT_IMAGE_VALUE_EQ(A, B) \
  { \
    cv::Mat _a = (cv::Mat)A; \
    cv::Mat _b = (cv::Mat)B; \
    auto _size = _a.total() * _a.elemSize(); \
    for (size_t i = 0; i < _size; ++i) { \
      ASSERT_EQ(_a.data[i], _b.data[i]); \
    } \
  }

#define ASSERT_MAT_IMAGE_EQ(A, B) \
  { \
    ASSERT_MAT_IMAGE_SIZE_EQ(A, B) \
    ASSERT_MAT_IMAGE_VALUE_EQ(A, B) \
  }

TEST(MatImageTest, RawImageConversion) {
  auto a = shisen_opencv::MatImage(cv::Mat(3, 2, CV_8UC3, cv::Scalar(32)));
  auto b = shisen_opencv::MatImage((shisen_cpp::RawImage)a);

  ASSERT_MAT_IMAGE_EQ(a, b);
}

TEST(MatImageTest, GrayRawImageConversion) {
  auto a = shisen_opencv::MatImage(cv::Mat(3, 2, CV_8UC1, cv::Scalar(64)));
  auto b = shisen_opencv::MatImage((shisen_cpp::RawImage)a);

  ASSERT_MAT_IMAGE_EQ(a, b);
}

TEST(MatImageTest, CompressedImageConversion) {
  auto a = shisen_opencv::MatImage(cv::Mat(3, 2, CV_8UC3, cv::Scalar(96)));
  auto b = shisen_opencv::MatImage((shisen_cpp::CompressedImage)a);

  ASSERT_MAT_IMAGE_SIZE_EQ(a, b);
}
