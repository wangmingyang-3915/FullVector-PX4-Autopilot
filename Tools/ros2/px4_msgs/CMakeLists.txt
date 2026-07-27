cmake_minimum_required(VERSION 3.8)
project(px4_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

file(GLOB px4_message_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "msg/*.msg")
list(SORT px4_message_files)

rosidl_generate_interfaces(${PROJECT_NAME} ${px4_message_files})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
