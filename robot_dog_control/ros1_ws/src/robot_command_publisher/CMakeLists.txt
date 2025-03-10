cmake_minimum_required(VERSION 3.0.2)
project(robot_command_publisher)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(command_publisher src/command_publisher.cpp)
target_link_libraries(command_publisher ${catkin_LIBRARIES}) 