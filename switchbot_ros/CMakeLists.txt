cmake_minimum_required(VERSION 2.8.3)
project(switchbot_ros)

find_package(catkin REQUIRED COMPONENTS
  catkin_virtualenv
  std_msgs
  message_generation
  )

add_message_files(
  FILES
  CommandTopic.msg
)

add_service_files(
  FILES
  Command.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime
)

catkin_generate_virtualenv(
  PYTHON_VERSION 3
)

catkin_install_python(
  PROGRAMS scripts/switchbot_ros.py scripts/switchbot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

