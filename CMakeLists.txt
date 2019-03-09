cmake_minimum_required(VERSION 2.8.3)
project(face_oracle)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  roboy_cognition_msgs
)


catkin_package(
  CATKIN_DEPENDS rospy std_msgs  roboy_cognition_msgs
)
