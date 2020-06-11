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

  if (rcutils_logging_set_logger_level(camera_node->get_logger().get_name(),
    RCUTILS_LOG_SEVERITY_INFO) != RCUTILS_RET_OK)
  {
    RCUTILS_LOG_FATAL("could not set the logger level");
  }

  rclcpp::spin(camera_node);

  rclcpp::shutdown();

  return 0;
}