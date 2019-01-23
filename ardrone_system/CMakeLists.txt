cmake_minimum_required(VERSION 2.8.3)
project(ardrone_system)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  geometry_msgs
)
find_package(Curses REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(move src/ardrone.cpp)
target_link_libraries(move
  ${catkin_LIBRARIES}
  ${CURSES_LIBRARIES}
)
