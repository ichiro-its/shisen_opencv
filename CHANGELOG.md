# Version 0.1.0 (11/06/2020)

## Alfi Maulana

- Create `shisen::Camera`.
  - a `rclcpp::Node` class.
  - capture an image using `cv::VideoCapture`.
  - encode the captured image using `cv::imencode()`.
  - publish it as an `sensor_msgs::msg::CompressedImage`.
- Create `shisen::Compressed
  - a `rclcpp::Node` class.
  - subscribe to an `sensor_msgs::msg::CompresssedImage`.
  - decode the received image using `cv::imdecode()`.
  - show the captured image using `cv::imshow()`.