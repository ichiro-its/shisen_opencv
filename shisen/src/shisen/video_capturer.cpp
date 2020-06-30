#include "shisen/video_capturer.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <stdexcept>
#include <vector>

using namespace shisen;
using namespace std::chrono_literals;

const std::map<std::string, int> VideoCapturer::property_ids = {
  { "pos_msec", cv::CAP_PROP_POS_MSEC },
  { "pos_frames", cv::CAP_PROP_POS_FRAMES },
  { "pos_avi_ratio", cv::CAP_PROP_POS_AVI_RATIO },
  { "frame_width", cv::CAP_PROP_FRAME_WIDTH },
  { "frame_height", cv::CAP_PROP_FRAME_HEIGHT },
  { "fps", cv::CAP_PROP_FPS },
  { "fourcc", cv::CAP_PROP_FOURCC },
  { "frame_count", cv::CAP_PROP_FRAME_COUNT },
  { "format", cv::CAP_PROP_FORMAT },
  { "mode", cv::CAP_PROP_MODE },
  { "brightness", cv::CAP_PROP_BRIGHTNESS },
  { "contrast", cv::CAP_PROP_CONTRAST },
  { "saturation", cv::CAP_PROP_SATURATION },
  { "hue", cv::CAP_PROP_HUE },
  { "gain", cv::CAP_PROP_GAIN },
  { "exposure", cv::CAP_PROP_EXPOSURE },
  { "convert_rgb", cv::CAP_PROP_CONVERT_RGB },
  { "white_balance_blue_u", cv::CAP_PROP_WHITE_BALANCE_BLUE_U },
  { "rectification", cv::CAP_PROP_RECTIFICATION },
  { "monochrome", cv::CAP_PROP_MONOCHROME },
  { "sharpness", cv::CAP_PROP_SHARPNESS },
  { "auto_exposure", cv::CAP_PROP_AUTO_EXPOSURE },
  { "gamma", cv::CAP_PROP_GAMMA },
  { "temperature", cv::CAP_PROP_TEMPERATURE },
  { "trigger", cv::CAP_PROP_TRIGGER },
  { "trigger_delay", cv::CAP_PROP_TRIGGER_DELAY },
  { "white_balance_red_v", cv::CAP_PROP_WHITE_BALANCE_RED_V },
  { "zoom", cv::CAP_PROP_ZOOM },
  { "focus", cv::CAP_PROP_FOCUS },
  { "guid", cv::CAP_PROP_GUID },
  { "iso_speed", cv::CAP_PROP_ISO_SPEED },
  { "backlight", cv::CAP_PROP_BACKLIGHT },
  { "pan", cv::CAP_PROP_PAN },
  { "tilt", cv::CAP_PROP_TILT },
  { "roll", cv::CAP_PROP_ROLL },
  { "iris", cv::CAP_PROP_IRIS },
  { "settings", cv::CAP_PROP_SETTINGS },
  { "buffersize", cv::CAP_PROP_BUFFERSIZE },
  { "autofocus", cv::CAP_PROP_AUTOFOCUS },
  { "sar_num", cv::CAP_PROP_SAR_NUM },
  { "sar_den", cv::CAP_PROP_SAR_DEN },
  { "backend", cv::CAP_PROP_BACKEND },
  { "channel", cv::CAP_PROP_CHANNEL },
  { "auto_wb", cv::CAP_PROP_AUTO_WB },
  { "wb_temperature", cv::CAP_PROP_WB_TEMPERATURE },
  { "codec_pixel_format", cv::CAP_PROP_CODEC_PIXEL_FORMAT },
  { "bitrate", cv::CAP_PROP_BITRATE }
};

VideoCapturer::VideoCapturer(std::string node_name) : rclcpp::Node(node_name)
{
  // initialize the raw image publisher
  {
    using RawImage = shisen_interfaces::msg::RawImage;
    raw_image_publisher = this->create_publisher<RawImage>(
      node_name + "/raw_image", 10
    );

    RCLCPP_INFO_STREAM(get_logger(), "publishing raw image on "
      << raw_image_publisher->get_topic_name());
  }

  // initialize the compressed image publisher
  {
    using CompressedImage = shisen_interfaces::msg::CompressedImage;
    compressed_image_publisher = this->create_publisher<CompressedImage>(
      node_name + "/compressed_image", 10
    );

    RCLCPP_INFO_STREAM(get_logger(), "publishing compressed image on "
      << compressed_image_publisher->get_topic_name());
  }

  // initialize the property event publisher
  {
    using PropertyEvent = shisen_interfaces::msg::PropertyEvent;
    property_event_publisher = this->create_publisher<PropertyEvent>(
      node_name + "/property_event", 10
    );

    RCLCPP_INFO_STREAM(get_logger(), "publishing property event on "
      << property_event_publisher->get_topic_name());
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
            for (auto &prop : property_map) {
              auto id = property_ids.find(prop.first);
              if (id == property_ids.end())
                continue;

              double value = video_capture.get(id->second);
              if (value != prop.second) {
                shisen_interfaces::msg::Property property;
                property.name = prop.first;
                property.value = prop.second = value;

                message.changed.push_back(property);
              }
            }

            // only publish the event message if there are some changes
            if (message.changed.size() > 0) {
              property_event_publisher->publish(message);
            }
          }

          // respond with all the properties
          for (auto &prop : property_map) {
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
            if (prop == property_map.end())
              continue;

            auto id = property_ids.find(property.name);
            if (id == property_ids.end())
              continue;

            // update the properties
            video_capture.set(id->second, property.value);
            double value = video_capture.get(id->second);

            if (value != prop->second) {
              property.value = prop->second = value;
              message.changed.push_back(property);
            }
          }

          // only publish the event message if there are some changes
          if (message.changed.size() > 0) {
            property_event_publisher->publish(message);
          }
        }
      }
    );
  }

  // initialize the capture timer
  capture_timer = this->create_wall_timer(30ms, [this]() {
    if (!video_capture.isOpened())
      return;

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
        cv::imencode(".jpg", captured_frame, bytes, { cv::IMWRITE_JPEG_QUALITY, 50 });
        message.data = bytes;

        compressed_image_publisher->publish(message);
      }

      RCLCPP_DEBUG_ONCE(get_logger(), "once, image captured and published");
    }
    else {
      RCLCPP_WARN_ONCE(get_logger(), "once, empty image captured");
    }
  });
}

VideoCapturer::~VideoCapturer()
{
  if (video_capture.isOpened()) {
    close();
  }
}

bool VideoCapturer::open(std::string file_name)
{
  if (!video_capture.isOpened()) {
    if (video_capture.open(file_name)) {
      RCLCPP_INFO_STREAM(get_logger(), "video capture opened on " << file_name);

      auto old_property_map = property_map;
      property_map.clear();

      // check available properties for the file
      for (auto &prop : property_ids) {
        try {
          // if set return true, property is available
          if (video_capture.set(prop.second, video_capture.get(prop.second))) {
            property_map.emplace(prop.first, video_capture.get(prop.second));
          }
        }
        catch (...) {
        }
      }

      // publish property event
      {
        shisen_interfaces::msg::PropertyEvent message;

        // list changed properties
        for (auto &prop : property_map) {
          shisen_interfaces::msg::Property property;
          property.name = prop.first;
          property.value = prop.second;

          message.changed.push_back(property);
        }

        // list deleted properties
        for (auto &prop : old_property_map) {
          if (property_map.find(prop.first) == property_map.end()) {
            message.deleted.push_back(prop.first);
          }
        }

        property_event_publisher->publish(message);
      }

      return true;
    }
    else {
      RCLCPP_ERROR_STREAM(get_logger(), "unable to open video capture on " << file_name);
    }
  }
  else {
    RCLCPP_WARN(get_logger(), "video capture had been opened");
  }

  return false;
}

bool VideoCapturer::close()
{
  if (video_capture.isOpened()) {
    video_capture.release();
    RCLCPP_INFO(get_logger(), "video capture closed");

    property_map.clear();

    return true;
  }
  else {
    RCLCPP_WARN(get_logger(), "video capture had not been opened");
  }

  return false;
}