cmake_minimum_required(VERSION 3.5)
project(quantum_camera_cpp)


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV 4.2.0 REQUIRED)
find_package(cv_bridge REQUIRED)


add_executable(camera_info src/camera_info.cpp)
ament_target_dependencies(camera_info rclcpp std_msgs sensor_msgs cv_bridge OpenCV)


install(TARGETS
  camera_info
  DESTINATION lib/${PROJECT_NAME})

ament_package()
