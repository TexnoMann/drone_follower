cmake_minimum_required(VERSION 2.8.3)
project(ar_cv)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# Debug option
 ADD_DEFINITIONS(-DDEBUG)

find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    roscpp
    rospy
    roslib
    sensor_msgs
    message_generation
)
find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS system thread REQUIRED)

## Generate messages in the 'msg' folder
add_message_files (
    FILES
    CircleInfo.msg
)


generate_messages (
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package (
    INCLUDE_DIRS include
    LIBRARIES ar_cv
    CATKIN_DEPENDS message_runtime
)

include_directories (
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(ar_cv src/SimpleCV.cpp)
target_link_libraries(ar_cv
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
)

add_executable(main_test test/MainTest.cpp)
target_link_libraries(main_test ar_cv)


#install binary and lib
install(TARGETS ar_cv
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#copy header files
install(DIRECTORY include/ar_cv/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
