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

install(PROGRAMS scripts/tweet.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
