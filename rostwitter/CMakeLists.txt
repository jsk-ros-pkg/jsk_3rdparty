cmake_minimum_required(VERSION 2.8.3)
project(rostwitter)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  mk
)

catkin_python_setup()

add_service_files(
  FILES
  Tweet.srv
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime
)

install(PROGRAMS scripts/tweet.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
