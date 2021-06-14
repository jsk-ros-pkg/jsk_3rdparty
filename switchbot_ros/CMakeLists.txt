cmake_minimum_required(VERSION 3.0.2)
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
