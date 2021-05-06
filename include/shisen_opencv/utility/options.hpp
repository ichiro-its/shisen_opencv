// Copyright (c) 2021 ICHIRO ITS
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

#ifndef SHISEN_OPENCV__UTILITY__OPTIONS_HPP_
#define SHISEN_OPENCV__UTILITY__OPTIONS_HPP_

#include <string>

namespace shisen_opencv
{

struct CombinedMatOptions
{
  bool enable_raw_image;
  bool enable_compressed_image;

  CombinedMatOptions()
  : enable_raw_image(true),
    enable_compressed_image(true)
  {
  }
};

struct CameraCaptureOptions
{
  std::string camera_file_name;
  int capture_fps;

  CameraCaptureOptions()
  : camera_file_name("/dev/video0"),
    capture_fps(60)
  {
  }
};

}  // namespace shisen_opencv

#endif  // SHISEN_OPENCV__UTILITY__OPTIONS_HPP_