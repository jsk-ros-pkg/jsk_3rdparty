cmake_minimum_required(VERSION 2.8.3)
project(rostwitter)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  rospy
  std_msgs
  mk
)

catkin_python_setup()

add_action_files(
  DIRECTORY action
  FILES Tweet.action
)


generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

# on noetic it needs catkin_install_python to support Python3
# but it does not work on indigo for some reason...
if($ENV{ROS_DISTRO} STREQUAL "indigo")
  install(DIRECTORY scripts
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    USE_SOURCE_PERMISSIONS
  )
else()
  file(GLOB SCRIPT_PROGRAMS scripts/*.py)
  catkin_install_python(
    PROGRAMS ${SCRIPT_PROGRAMS}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts/
  )
endif()

install(DIRECTORY test launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/rostwitter.test)
endif()
