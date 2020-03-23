cmake_minimum_required(VERSION 2.8.3)
project(audio_stream)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  audio_stream_msgs
  roscpp
  rospy
)

catkin_package(
  LIBRARIES audio_stream
  CATKIN_DEPENDS audio_stream_msgs roscpp rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
