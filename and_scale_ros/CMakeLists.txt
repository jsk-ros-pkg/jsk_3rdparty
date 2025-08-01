cmake_minimum_required(VERSION 2.8.3)
project(and_scale_ros)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
)

add_message_files(
  FILES
  Weight.msg
  WeightStamped.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
)

file(GLOB PYTHON_SCRIPTS scripts/*.py)
if($ENV{ROS_DISTRO} STREQUAL "indigo") # on noetic it needs catkin_install_python to support Python3 and it does not work on indigo for some reason...
  install(PROGRAMS ${PYTHON_SCRIPTS}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
else()
  catkin_install_python(PROGRAMS ${PYTHON_SCRIPTS}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
