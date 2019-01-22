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
include_directories(${CURSES_INCLUDE_DIR})





## Generate messages in the 'msg' folder
add_message_files(
  FILES
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )


## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package(CATKIN_DEPENDS
	message_runtime
#  INCLUDE_DIRS include
#  LIBRARIES test_kinematics
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(move src/ardrone.cpp)
target_link_libraries(move ${catkin_LIBRARIES})
target_link_libraries(move ${CURSES_LIBRARIES})
