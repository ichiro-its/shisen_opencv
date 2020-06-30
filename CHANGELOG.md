# Version 0.4.0

- Add interfaces that handle properties in the `shisen_interfaces`.
- Change `shisen::Camera` to `shisen::VideoCapturer`.
  - add boolean return in `open()` and `close()`.
  - video capture properties will be handled by message and service properties
    instead of ROS 2 parameter.
- Merge `shisen::CompressedViewer` and `shisen::RawViewer`
  into `shisen::ImageViewer`.

# Version 0.3.0 (30/06/2020)

## Alfi Maulana

- Separate package to be `shisen` and `shisen_interfaces`.
  - `shisen` contains the main tools implementation.
  - `shisen_interfaces` contains ros 2 interfaces used by `shisen`.
- Add `shisen_interfaces` that contains interfaces that handle image transport.
- Replace `sensor_msgs::msg::CompressedImage` with `shisen_interfaces::msg::CompressedImage`.
- Add ability to publish raw image in the `shisen::Camera`
  using an `shisen_interfaces::msg::RawImage`.
- Add `shisen::RawViewer` class.
  - subscribe to an `shisen_interfaces::msg::RawImage`.
  - work just like `shisen::CompressedViewer` but using raw data
    instead of the encoded one.

# Version 0.2.1 (23/06/2020)

## Alfi Maulana

- Modify `shisen::Camera`
  - use `on_set_parameters_callback` instead of `parameter_event` subscription
    to verify changes of parameter values.

# Version 0.2.0 (16/06/2020)

## Alfi Maulana

- Modify `shisen::Camera`.
  - add ability to close the camera device.
  - add ability to change the camera properties.
  - modify the camera properties using parameters.

# Version 0.1.0 (11/06/2020)

## Alfi Maulana

- Create `shisen::Camera`.
  - a `rclcpp::Node` class.
  - capture an image using `cv::VideoCapture`.
  - encode the captured image using `cv::imencode()`.
  - publish it as an `sensor_msgs::msg::CompressedImage`.
- Create `shisen::CompressedViewer` class.
  - a `rclcpp::Node` class.
  - subscribe to an `sensor_msgs::msg::CompresssedImage`.
  - decode the received image using `cv::imdecode()`.
  - show the captured image using `cv::imshow()`.