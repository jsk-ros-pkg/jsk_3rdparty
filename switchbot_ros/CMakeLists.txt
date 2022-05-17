cmake_minimum_required(VERSION 2.8.3)
project(switchbot_ros)

find_package(
  catkin REQUIRED COMPONENTS
  message_generation
  actionlib_msgs
)

catkin_python_setup()

add_message_files(
  FILES
  Device.msg
  DeviceArray.msg
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

catkin_package()

include_directories()

# install
# euslisp
file(GLOB EUSLISP_SCRIPTS scripts/*.l)
install(FILES ${EUSLISP_SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
# python
file(GLOB PYTHON_SCRIPTS scripts/*.py)
catkin_install_python(
  PROGRAMS ${PYTHON_SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
# launch
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

