cmake_minimum_required(VERSION 3.5)
project(ros2_control_demo_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY carlikebot/urdf carlikebot/rviz
  DESTINATION share/${PROJECT_NAME}/carlikebot
)

ament_package()
