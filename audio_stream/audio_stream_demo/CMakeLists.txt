cmake_minimum_required(VERSION 2.8.3)
project(audio_stream_demo)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
)

catkin_package(
  LIBRARIES audio_stream_demo
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
