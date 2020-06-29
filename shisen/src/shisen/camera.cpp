#include "shisen/camera.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <stdexcept>
#include <vector>

using namespace shisen;
using namespace std::chrono_literals;

Camera::Camera(std::string node_name) : rclcpp::Node(node_name)
{
  for (auto pair : property_names) {
    this->declare_parameter(pair.first, 0.0);
  }

  on_set_parameter_handler = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> &parameters) {
      auto result = rcl_interfaces::msg::SetParametersResult();
      result.successful = true;

      try {
        auto &casted_parameters = const_cast<std::vector<rclcpp::Parameter> &>(parameters);
        for (auto &parameter : casted_parameters) {
          auto pair = property_names.find(parameter.get_name());
          if (pair == property_names.end())
            continue;

          if (!video_capture.isOpened())
            throw std::runtime_error("video capture had not been opened");

          RCLCPP_DEBUG_STREAM(get_logger(), "request to modify " << parameter.get_name()
            << " to " << parameter.value_to_string());

          double value = 0.0;
          switch (parameter.get_type()) {
            case rclcpp::ParameterType::PARAMETER_DOUBLE:
              value = parameter.as_double();
              break;

            case rclcpp::ParameterType::PARAMETER_INTEGER:
              value = (double)parameter.as_int();
              break;

            default:
              throw std::runtime_error("invalid parameter type");
          }

          int property = pair->second;

          video_capture.set(property, value);
          value = video_capture.get(property);

          parameter = rclcpp::Parameter(parameter.get_name(), value);

          RCLCPP_DEBUG_STREAM(get_logger(), parameter.get_name()
            << " modified to " << parameter.value_to_string());
        }
      }
      catch (std::exception &e) {
        RCLCPP_WARN(get_logger(), e.what());
        result.reason = e.what();
        result.successful = false;
      }

      return result;
    }
  );

  raw_image_publisher = this->create_publisher<shisen_interfaces::msg::RawImage>(
    node_name + "/raw_image", 10
  );

  RCLCPP_INFO_STREAM(get_logger(), "publishing raw image on "
    << raw_image_publisher->get_topic_name());

  compressed_image_publisher = this->create_publisher<shisen_interfaces::msg::CompressedImage>(
    node_name + "/compressed_image", 10
  );

  RCLCPP_INFO_STREAM(get_logger(), "publishing compressed image on "
    << compressed_image_publisher->get_topic_name());

  capture_timer = this->create_wall_timer(30ms, [this]() {
    if (!video_capture.isOpened())
      return;

    cv::Mat captured_frame;
    video_capture.read(captured_frame);

    if (!captured_frame.empty()) {
      {
        shisen_interfaces::msg::RawImage message;

        message.type = captured_frame.type();
        message.cols = captured_frame.cols;
        message.rows = captured_frame.rows;

        auto size = captured_frame.total() * captured_frame.elemSize();
        message.data.assign(captured_frame.data, captured_frame.data + size);

        raw_image_publisher->publish(message);
      }

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

Camera::~Camera()
{
  close();
}

void Camera::open(std::string device_name)
{
  if (!video_capture.isOpened()) {
    if (video_capture.open(device_name)) {
      RCLCPP_INFO_STREAM(get_logger(), "video capture opened on " << device_name);

      for (auto pair : property_names) {
        double value = video_capture.get(pair.second);
        this->set_parameter(rclcpp::Parameter(pair.first, value));
      }
    }
    else {
      RCLCPP_ERROR_STREAM(get_logger(), "unable to open video capture on " << device_name);
    }
  }
  else {
    RCLCPP_WARN(get_logger(), "video capture had been opened");
  }
}

void Camera::open()
{
  open("/dev/video0");
}

void Camera::close()
{
  if (video_capture.isOpened()) {
    video_capture.release();
    RCLCPP_INFO(get_logger(), "video capture closed");
  }
  else {
    RCLCPP_WARN(get_logger(), "video capture had not been opened");
  }
}