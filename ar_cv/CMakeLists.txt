cmake_minimum_required(VERSION 2.8.3)
project(ar_cv)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# Debug option
# ADD_DEFINITIONS(-DDEBUG)

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
    # INCLUDE_DIRS include
    # LIBRARIES ar_cv
    CATKIN_DEPENDS message_runtime
    # DEPENDS system_lib
)

include_directories (
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(main_test test/MainTest.cpp src/SimpleCV.cpp)
target_link_libraries(main_test
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
)