#include <rclcpp/rclcpp.hpp>
#include <shisen/camera.hpp>

#include <memory>
#include <string>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto camera_node = std::make_shared<shisen::Camera>("camera");

  if (argc > 1)
    camera_node->open(argv[1]);
  else
    camera_node->open();

  rclcpp::spin(camera_node);

  rclcpp::shutdown();

  return 0;
}