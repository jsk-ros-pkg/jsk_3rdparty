cmake_minimum_required(VERSION 2.8.3)

project(spatial_audio_demo)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  audio_stream_msgs
  interactive_markers
  roscpp
  rospy
  spatial_audio_msgs
  tf2
  tf2_ros
  visualization_msgs
)

find_package(Boost REQUIRED)

catkin_package(
  CATKIN_DEPENDS
      audio_stream_msgs
      interactive_markers
      roscpp
      rospy
      spatial_audio_msgs
      tf2
      tf2_ros
      visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#########
# Build #
#########
add_executable(audio_source_interactive_marker
    src/audio_source_interactive_marker.cpp
)
target_link_libraries(audio_source_interactive_marker
    ${catkin_LIBRARIES}
)
