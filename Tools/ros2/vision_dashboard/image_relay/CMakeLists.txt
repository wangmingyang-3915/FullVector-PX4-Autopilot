cmake_minimum_required(VERSION 3.8)
project(vision_dashboard_image_relay)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

add_executable(image_relay src/image_relay.cpp)
ament_target_dependencies(image_relay
  rclcpp apriltag_msgs px4_msgs sensor_msgs std_msgs tf2_msgs)

install(TARGETS image_relay DESTINATION lib/${PROJECT_NAME})
ament_package()
