cmake_minimum_required(VERSION 3.0.2)
project(dialogflow_intent_client)

find_package(catkin REQUIRED COMPONENTS
  catkin_virtualenv REQUIRED
  message_generation
  std_msgs
  actionlib_msgs
  )

add_message_files(
  FILES
  IntentInfo.msg
  )

add_action_files(
  FILES
  RegisterIntent.action
  ListIntent.action
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
  )

catkin_package(
  CATKIN_DEPENDS message_runtime
  )

catkin_generate_virtualenv(
  PYTHON_INTERPRETER python3
  )

file(GLOB NODE_SCRIPTS_FILES node_scripts/*.py)

catkin_install_python(
  PROGRAMS ${NODE_SCRIPTS_FILES}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
