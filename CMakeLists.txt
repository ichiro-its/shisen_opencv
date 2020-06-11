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
find_package(sensor_msgs REQUIRED)

add_library(${PROJECT_NAME}
  "src/${PROJECT_NAME}/camera.cpp"
  "src/${PROJECT_NAME}/compressed_viewer.cpp"
)

target_include_directories(${PROJECT_NAME} PUBLIC "include")

ament_target_dependencies(${PROJECT_NAME}
  OpenCV
  rclcpp
  sensor_msgs
)

add_executable(camera "src/camera_main.cpp")
target_link_libraries(camera ${PROJECT_NAME})

add_executable(compressed_viewer "src/compressed_viewer_main.cpp")
target_link_libraries(compressed_viewer ${PROJECT_NAME})

install(DIRECTORY "include" DESTINATION ".")
install(TARGETS ${PROJECT_NAME} DESTINATION "lib")
install(TARGETS camera compressed_viewer DESTINATION "lib/${PROJECT_NAME}")

ament_export_include_directories("include")
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp)
ament_package()
