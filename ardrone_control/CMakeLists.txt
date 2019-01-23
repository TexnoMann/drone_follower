cmake_minimum_required(VERSION 2.8.3)
project(ardrone_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  geometry_msgs
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

catkin_package(
  LIBRARIES ardrone_control
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(ardrone_control src/DroneController.cpp)
target_link_libraries(ardrone_control
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test1 tests/test.cpp)
target_link_libraries(test1 ardrone_control)

#install binary and lib
install(TARGETS ardrone_control
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#copy header files
install(DIRECTORY include/ardrone_control/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)