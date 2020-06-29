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

  rclcpp::spin(compressed_viewer);

  rclcpp::shutdown();

  return 0;
}