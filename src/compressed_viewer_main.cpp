#include <rclcpp/rclcpp.hpp>
#include <shisen/compressed_viewer.hpp>

#include <memory>
#include <string>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto compressed_viewer = std::make_shared<shisen::CompressedViewer>(
    "compressed_viewer", (argc > 1) ? argv[1] : "camera/compressed_image"
  );

  if (rcutils_logging_set_logger_level(compressed_viewer->get_logger().get_name(),
    RCUTILS_LOG_SEVERITY_INFO) != RCUTILS_RET_OK)
  {
    RCUTILS_LOG_FATAL("could not set the logger level");
  }

  rclcpp::spin(compressed_viewer);

  rclcpp::shutdown();

  return 0;
}