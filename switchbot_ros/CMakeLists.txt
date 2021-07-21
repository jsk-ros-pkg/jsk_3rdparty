cmake_minimum_required(VERSION 2.8.3)
project(switchbot_ros)

find_package(
  catkin REQUIRED COMPONENTS
  actionlib_msgs
)

add_action_files(
  FILES
  SwitchBotCommand.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)
