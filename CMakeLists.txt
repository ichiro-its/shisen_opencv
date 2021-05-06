cmake_minimum_required(VERSION 3.5)

project(shisen_opencv)

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
find_package(shisen_cpp REQUIRED)

install(DIRECTORY "include" DESTINATION ".")

add_library(${PROJECT_NAME}
  # "src/node/camera_capture.cpp"
  # "src/node/camera.cpp"
  # "src/node/viewer.cpp"
  "src/utility/mat_image.cpp")

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}
  OpenCV rclcpp shisen_cpp)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION "lib"
  LIBRARY DESTINATION "lib"
  RUNTIME DESTINATION "bin")

# add_executable(camera "bin/camera.cpp")
# add_executable(viewer "bin/viewer.cpp")

# foreach(target camera viewer)
#   target_link_libraries(${target} ${PROJECT_NAME})

#   install(TARGETS ${target}
#     DESTINATION "lib/${PROJECT_NAME}")
# endforeach()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_tests
    # "test/compile_test.cpp"
    "test/mat_image_test.cpp")

  target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME})

  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(OpenCV rclcpp shisen_cpp)
ament_export_include_directories("include")
ament_export_libraries(${PROJECT_NAME})

ament_package()
