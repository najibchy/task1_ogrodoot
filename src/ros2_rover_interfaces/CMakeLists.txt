cmake_minimum_required(VERSION 3.5)
project(ros2_rover_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PickDeliver.action"
  "srv/CaptureImage.srv"
  DEPENDENCIES
    geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()