cmake_minimum_required(VERSION 2.8.3)
project(audio_stream_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
)

add_message_files(
  FILES
  AudioData.msg
  AudioInfo.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs
)
