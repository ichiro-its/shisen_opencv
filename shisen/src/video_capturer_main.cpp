#include <rclcpp/rclcpp.hpp>
#include <shisen/video_capturer.hpp>

#include <memory>
#include <string>

int main(int argc, char **argv)
{
  if (argc < 2) {
    std::cout << "Usage: ros2 run shisen video_capturer <file_name>" << std::endl;
    return 1;
  }

  std::string file_name = argv[1];

  rclcpp::init(argc, argv);

  auto video_capturer = std::make_shared<shisen::VideoCapturer>("video_capturer");

  if (video_capturer->open(file_name)) {
    rclcpp::spin(video_capturer);
  }

  rclcpp::shutdown();

  return 0;
}