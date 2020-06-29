#include <rclcpp/rclcpp.hpp>
#include <shisen/raw_viewer.hpp>

#include <memory>
#include <string>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto raw_viewer = std::make_shared<shisen::RawViewer>(
    "raw_viewer", (argc > 1) ? argv[1] : "camera/raw_image"
  );

  rclcpp::spin(raw_viewer);

  rclcpp::shutdown();

  return 0;
}