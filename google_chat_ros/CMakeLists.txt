cmake_minimum_required(VERSION 2.8.3)
project(google_chat_ros)

find_package(
  catkin REQUIRED COMPONENTS
  actionlib_msgs
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package()

include_directories()
