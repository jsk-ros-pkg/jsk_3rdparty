cmake_minimum_required(VERSION 2.8.3)

project(spatial_audio_demo)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  audio_common_msgs
  interactive_markers
  roscpp
  rospy
  spatial_audio_msgs
  tf2
  tf2_ros
  visualization_msgs
)

find_package(Boost REQUIRED)

catkin_python_setup()

generate_dynamic_reconfigure_options(
    cfg/BeaconSoundPublisher.cfg
)

catkin_package(
  CATKIN_DEPENDS
      audio_common_msgs
      interactive_markers
      roscpp
      rospy
      spatial_audio
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

#########
# Install #
#########
catkin_install_python(
    PROGRAMS node_scripts/sound_navigator
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#########
# Testing #
#########
if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest(test/test_spatial_audio_with_auto_play.test)
    add_rostest(test/test_spatial_audio_without_auto_play.test)
endif()
