cmake_minimum_required(VERSION 3.8)
project(mediapipe_ros2_node)

find_package(ament_cmake REQUIRED)

install(DIRECTORY models DESTINATION share/${PROJECT_NAME} OPTIONAL)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME} OPTIONAL)
install(DIRECTORY rviz   DESTINATION share/${PROJECT_NAME} OPTIONAL)

ament_package()
