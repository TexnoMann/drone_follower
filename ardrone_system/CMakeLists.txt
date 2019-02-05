cmake_minimum_required(VERSION 2.8.3)
project(ardrone_system)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  ardrone_control
  ardrone_autonomy
  ar_cv
  cv_bridge

)

# Debug option
 ADD_DEFINITIONS(-DDEBUG)
 
find_package(Curses REQUIRED)

find_package(OpenCV REQUIRED)

find_package(Boost COMPONENTS system thread REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ardrone src/ardrone.cpp)
add_executable(test_cv src/test_cv.cpp)
target_link_libraries(ardrone
  ${ar_cv_LIBRARIES}
  ${ardrone_control_LIBRARIES}
  ${catkin_LIBRARIES}
  ${CURSES_LIBRARIES}
)

target_link_libraries(test_cv
  ${ar_cv_LIBRARIES}
  ${ardrone_control_LIBRARIES}
  ${catkin_LIBRARIES}
  ${CURSES_LIBRARIES}
)

add_executable(convenience src/convenience.cpp)
target_link_libraries(convenience
  ${catkin_LIBRARIES}
  ${CURSES_LIBRARIES}
)

add_executable(graph src/graph.cpp)
target_link_libraries(graph
  ${ar_cv_LIBRARIES}
  ${ardrone_control_LIBRARIES}
  ${catkin_LIBRARIES}
  ${CURSES_LIBRARIES}
)