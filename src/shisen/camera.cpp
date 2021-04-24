// Copyright 2020-2021 ICHIRO ITS
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

#include "shisen/camera.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace shisen
{

const std::map<std::string, int> Camera::property_ids = {
  // preprocess Effect
  {"brightness", cv::CAP_PROP_BRIGHTNESS},
  {"contrast", cv::CAP_PROP_CONTRAST},
  {"saturation", cv::CAP_PROP_SATURATION},
  {"hue", cv::CAP_PROP_HUE},
  {"gain", cv::CAP_PROP_GAIN},
  {"exposure", cv::CAP_PROP_EXPOSURE},
  {"sharpness", cv::CAP_PROP_SHARPNESS},
  {"temperature", cv::CAP_PROP_TEMPERATURE},

  // capture setting
  {"fps", cv::CAP_PROP_FPS},
  {"backlight", cv::CAP_PROP_BACKLIGHT},
  {"autofocus", cv::CAP_PROP_AUTOFOCUS},
  {"auto_wb", cv::CAP_PROP_AUTO_WB},
  {"auto_exposure", cv::CAP_PROP_AUTO_EXPOSURE},

  // frame size
  {"frame_width", cv::CAP_PROP_FRAME_WIDTH},
  {"frame_height", cv::CAP_PROP_FRAME_HEIGHT},

  // camera transform
  {"focus", cv::CAP_PROP_FOCUS},
  {"zoom", cv::CAP_PROP_ZOOM},
  {"pan", cv::CAP_PROP_PAN},
  {"tilt", cv::CAP_PROP_TILT},
  {"roll", cv::CAP_PROP_ROLL},

  // unused for now
  // {"fourcc", cv::CAP_PROP_FOURCC},
  // {"frame_count", cv::CAP_PROP_FRAME_COUNT},
  // {"format", cv::CAP_PROP_FORMAT},
  // {"mode", cv::CAP_PROP_MODE},
  // {"convert_rgb", cv::CAP_PROP_CONVERT_RGB},
  // {"white_balance_blue_u", cv::CAP_PROP_WHITE_BALANCE_BLUE_U},
  // {"rectification", cv::CAP_PROP_RECTIFICATION},
  // {"monochrome", cv::CAP_PROP_MONOCHROME},
  // {"gamma", cv::CAP_PROP_GAMMA},
  // {"trigger", cv::CAP_PROP_TRIGGER},
  // {"trigger_delay", cv::CAP_PROP_TRIGGER_DELAY},
  // {"white_balance_red_v", cv::CAP_PROP_WHITE_BALANCE_RED_V},
  // {"guid", cv::CAP_PROP_GUID},
  // {"iso_speed", cv::CAP_PROP_ISO_SPEED},
  // {"iris", cv::CAP_PROP_IRIS},
  // {"settings", cv::CAP_PROP_SETTINGS},
  // {"buffersize", cv::CAP_PROP_BUFFERSIZE},
  // {"sar_num", cv::CAP_PROP_SAR_NUM},
  // {"sar_den", cv::CAP_PROP_SAR_DEN},
  // {"backend", cv::CAP_PROP_BACKEND},
  // {"channel", cv::CAP_PROP_CHANNEL},
  // {"wb_temperature", cv::CAP_PROP_WB_TEMPERATURE},
  // {"codec_pixel_format", cv::CAP_PROP_CODEC_PIXEL_FORMAT},
  // {"bitrate", cv::CAP_PROP_BITRATE}
};

Camera::Camera(std::string node_name)
: rclcpp::Node(node_name)
{
  // initialize the raw image publisher
  {
    using RawImage = shisen_interfaces::msg::RawImage;
    raw_image_publisher = this->create_publisher<RawImage>(
      node_name + "/raw_image", 10
    );

    RCLCPP_INFO_STREAM(
      get_logger(), "publishing raw image on " <<
        raw_image_publisher->get_topic_name());
  }

  // initialize the compressed image publisher
  {
    using CompressedImage = shisen_interfaces::msg::CompressedImage;
    compressed_image_publisher = this->create_publisher<CompressedImage>(
      node_name + "/compressed_image", 10
    );

    RCLCPP_INFO_STREAM(
      get_logger(), "publishing compressed image on " <<
        compressed_image_publisher->get_topic_name());
  }

  // initialize the property event publisher
  {
    using PropertyEvent = shisen_interfaces::msg::PropertyEvent;
    property_event_publisher = this->create_publisher<PropertyEvent>(
      node_name + "/property_event", 10
    );

    RCLCPP_INFO_STREAM(
      get_logger(), "publishing property event on " <<
        property_event_publisher->get_topic_name());
  }

  // initialize the get properties service
  {
    using GetProperties = shisen_interfaces::srv::GetProperties;
    get_properties_service = this->create_service<GetProperties>(
      node_name + "/get_properties",
      [this](std::shared_ptr<GetProperties::Request> request,
      std::shared_ptr<GetProperties::Response> response) {
        (void)request;

        if (video_capture.isOpened()) {
          // update properties and publish the event message
          {
            shisen_interfaces::msg::PropertyEvent message;
            for (auto & prop : property_map) {
              auto id = property_ids.find(prop.first);
              if (id == property_ids.end()) {
                continue;
              }

              shisen_interfaces::msg::Property property;
              property.name = prop.first;
              property.value = prop.second = video_capture.get(id->second);

              message.changed.push_back(property);
            }

            // only publish the event message if there are some changes
            if (message.changed.size() > 0) {
              property_event_publisher->publish(message);
            }
          }

          // respond with all the properties
          for (auto & prop : property_map) {
            shisen_interfaces::msg::Property property;
            property.name = prop.first;
            property.value = prop.second;

            response->properties.push_back(property);
          }
        }
      }
    );
  }

  // initialize the set properties service
  {
    using SetProperties = shisen_interfaces::srv::SetProperties;
    set_properties_service = this->create_service<SetProperties>(
      node_name + "/set_properties",
      [this](std::shared_ptr<SetProperties::Request> request,
      std::shared_ptr<SetProperties::Response> response) {
        (void)response;

        // update properties based on the request and publish the event message
        {
          shisen_interfaces::msg::PropertyEvent message;
          for (auto property : request->properties) {
            auto prop = property_map.find(property.name);
            if (prop == property_map.end()) {
              continue;
            }

            auto id = property_ids.find(property.name);
            if (id == property_ids.end()) {
              continue;
            }

            // update the properties
            video_capture.set(id->second, property.value);
            double value = video_capture.get(id->second);

            property.value = prop->second = value;
            message.changed.push_back(property);

            switch (id->second) {
              // changes on frame width would affect frame height
              case cv::CAP_PROP_FRAME_WIDTH: {
                shisen_interfaces::msg::Property property;
                property.name = "frame_height";
                property.value = video_capture.get(cv::CAP_PROP_FRAME_HEIGHT);
                message.changed.push_back(property);
                break;
              }

              // changes on frame height would affect frame width
              case cv::CAP_PROP_FRAME_HEIGHT: {
                shisen_interfaces::msg::Property property;
                property.name = "frame_width";
                property.value = video_capture.get(cv::CAP_PROP_FRAME_WIDTH);
                message.changed.push_back(property);
                break;
              }

              // changes on auto white balance would affect temperature
              case cv::CAP_PROP_AUTO_WB: {
                double value = video_capture.get(cv::CAP_PROP_TEMPERATURE);
                if (video_capture.set(cv::CAP_PROP_TEMPERATURE, value)) {
                  shisen_interfaces::msg::Property property;
                  property.name = "temperature";
                  property.value = property_map["temperature"] = value;
                  message.changed.push_back(property);
                } else {
                  property_map.erase("temperature");
                  message.deleted.push_back("temperature");
                }
                break;
              }

              // changes on auto focus would affect focus
              case cv::CAP_PROP_AUTOFOCUS: {
                double value = video_capture.get(cv::CAP_PROP_FOCUS);
                if (video_capture.set(cv::CAP_PROP_FOCUS, value)) {
                  shisen_interfaces::msg::Property property;
                  property.name = "focus";
                  property.value = property_map["focus"] = value;
                  message.changed.push_back(property);
                } else {
                  property_map.erase("focus");
                  message.deleted.push_back("focus");
                }
                break;
              }
            }
          }

          // only publish the event message if there are some changes
          if (message.changed.size() > 0 || message.deleted.size() > 0) {
            property_event_publisher->publish(message);
          }
        }
      }
    );
  }

  // initialize the capture timer
  capture_timer = this->create_wall_timer(
    30ms, [this]() {
      if (!video_capture.isOpened()) {
        return;
      }

      cv::Mat captured_frame;
      video_capture.read(captured_frame);

      if (!captured_frame.empty()) {
        // publish raw image
        {
          shisen_interfaces::msg::RawImage message;

          message.type = captured_frame.type();
          message.cols = captured_frame.cols;
          message.rows = captured_frame.rows;

          auto byte_size = captured_frame.total() * captured_frame.elemSize();
          message.data.assign(captured_frame.data, captured_frame.data + byte_size);

          raw_image_publisher->publish(message);
        }

        // publish compressed image
        {
          shisen_interfaces::msg::CompressedImage message;

          std::vector<unsigned char> bytes;
          cv::imencode(".jpg", captured_frame, bytes, {cv::IMWRITE_JPEG_QUALITY, 50});
          message.data = bytes;

          compressed_image_publisher->publish(message);
        }

        RCLCPP_DEBUG_ONCE(get_logger(), "once, image captured and published");
      } else {
        RCLCPP_WARN_ONCE(get_logger(), "once, empty image captured");
      }
    }
  );
}

Camera::~Camera()
{
  if (video_capture.isOpened()) {
    close();
  }
}

bool Camera::open(std::string file_name)
{
  if (!video_capture.isOpened()) {
    if (video_capture.open(file_name)) {
      RCLCPP_INFO_STREAM(get_logger(), "video capture opened on " << file_name);

      auto old_property_map = property_map;
      property_map.clear();

      // check available properties for the file
      for (auto & prop : property_ids) {
        try {
          // if set return true, property is available
          if (video_capture.set(prop.second, video_capture.get(prop.second))) {
            property_map.emplace(prop.first, video_capture.get(prop.second));
          }
        } catch (...) {
        }
      }

      // publish property event
      {
        shisen_interfaces::msg::PropertyEvent message;

        // list changed properties
        for (auto & prop : property_map) {
          shisen_interfaces::msg::Property property;
          property.name = prop.first;
          property.value = prop.second;

          message.changed.push_back(property);
        }

        // list deleted properties
        for (auto & prop : old_property_map) {
          if (property_map.find(prop.first) == property_map.end()) {
            message.deleted.push_back(prop.first);
          }
        }

        property_event_publisher->publish(message);
      }

      return true;
    } else {
      RCLCPP_ERROR_STREAM(get_logger(), "unable to open video capture on " << file_name);
    }
  } else {
    RCLCPP_WARN(get_logger(), "video capture had been opened");
  }

  return false;
}

bool Camera::close()
{
  if (video_capture.isOpened()) {
    video_capture.release();
    RCLCPP_INFO(get_logger(), "video capture closed");

    property_map.clear();

    return true;
  } else {
    RCLCPP_WARN(get_logger(), "video capture had not been opened");
  }

  return false;
}

}  // namespace shisen
