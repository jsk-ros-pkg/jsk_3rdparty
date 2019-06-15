cmake_minimum_required(VERSION 2.8.3)
project(dialogflow_ros)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
    geometry_msgs)

add_message_files(
  FILES
  Attention.msg
  DialogResponse.msg
)

add_service_files(
  FILES
  EnqueueTask.srv
  InterruptTask.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package()


install(DIRECTORY node_scripts 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
