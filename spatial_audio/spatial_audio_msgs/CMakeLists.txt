cmake_minimum_required(VERSION 2.8.3)
project(spatial_audio_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  message_generation
  std_msgs
)

add_message_files(
  FILES
  AudioSource.msg
  AudioSourceArray.msg
)

add_service_files(
  FILES
  AddSpatialAudio.srv
  UpdateSpatialAudio.srv
  TriggerSpatialAudio.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
