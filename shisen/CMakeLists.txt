cmake_minimum_required(VERSION 3.10)

project(shisen)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(shisen_interfaces REQUIRED)

add_library(${PROJECT_NAME}
  "src/${PROJECT_NAME}/video_capturer.cpp"
  "src/${PROJECT_NAME}/image_viewer.cpp"
)

target_include_directories(${PROJECT_NAME} PUBLIC "include")

ament_target_dependencies(${PROJECT_NAME}
  OpenCV
  rclcpp
  shisen_interfaces
)

add_executable(video_capturer "src/video_capturer_main.cpp")
target_link_libraries(video_capturer ${PROJECT_NAME})

add_executable(image_viewer "src/image_viewer_main.cpp")
target_link_libraries(image_viewer ${PROJECT_NAME})

install(DIRECTORY "include" DESTINATION ".")
install(TARGETS ${PROJECT_NAME} DESTINATION "lib")
install(TARGETS video_capturer image_viewer DESTINATION "lib/${PROJECT_NAME}")

ament_export_include_directories("include")
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp shisen_interfaces)
ament_package()
