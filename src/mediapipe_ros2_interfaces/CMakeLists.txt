cmake_minimum_required(VERSION 3.8)
project(mediapipe_ros2_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HandGesture.msg"
  "msg/Hand.msg"
  "msg/HandLandmarks.msg"
  "msg/FaceLandmarks.msg"
  "msg/PoseLandmarks.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
