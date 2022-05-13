cmake_minimum_required(VERSION 2.8.3)
project(dialogflow_task_executive)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
    actionlib_msgs
    dialogflow_ros
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
)

file(GLOB NODE_SCRIPTS_FILES node_scripts/*.py)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
