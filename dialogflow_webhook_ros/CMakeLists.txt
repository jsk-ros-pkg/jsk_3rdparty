cmake_minimum_required(VERSION 2.8.3)
project(dialogflow_webhook_ros)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
)

add_message_files(
  FILES
  OriginalDetectIntentRequest.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

catkin_install_python(
  PROGRAMS node_scripts/webhook_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
