cmake_minimum_required(VERSION 2.8.3)
project(google_chat_ros)

find_package(
  catkin REQUIRED COMPONENTS
  actionlib_msgs
)

catkin_python_setup()

add_action_files(
  FILES
  GoogleChat.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package()

include_directories()

# install
file(GLOB SCRIPTS scripts/*)

catkin_install_python(
  PROGRAMS ${SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

