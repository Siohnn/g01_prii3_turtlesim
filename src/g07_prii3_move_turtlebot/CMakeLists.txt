cmake_minimum_required(VERSION 3.5)
project(g07_prii3_move_turtlebot)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)

install(
  PROGRAMS
  src/move_turtlebot.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

