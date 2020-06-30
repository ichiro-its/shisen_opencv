#include <rclcpp/rclcpp.hpp>
#include <shisen/image_viewer.hpp>

#include <memory>
#include <string>

int main(int argc, char **argv)
{
  if (argc < 2) {
    std::cout << "Usage: ros2 run shisen image_viewer <topic_name>" << std::endl;
    return 1;
  }

  std::string topic_name = argv[1];

  rclcpp::init(argc, argv);

  auto image_viewer = std::make_shared<shisen::ImageViewer>(
    "image_viewer", topic_name
  );

  rclcpp::spin(image_viewer);

  rclcpp::shutdown();

  return 0;
}