cmake_minimum_required(VERSION 2.8.3)
project(dialogflow_task_executive)

find_package(catkin REQUIRED COMPONENTS
    catkin_virtualenv
    message_generation
    std_msgs
)

add_message_files(
  FILES
  DialogResponse.msg
  ConversationText.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

catkin_generate_virtualenv(
  PYTHON_VERSION 3
)

catkin_install_python(
  PROGRAMS node_scripts/webhook_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES node_scripts/dialogflow_client.py node_scripts/task_executive.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
