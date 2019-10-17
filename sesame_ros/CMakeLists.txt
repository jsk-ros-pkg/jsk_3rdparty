cmake_minimum_required(VERSION 2.8.3)
project(sesame_ros)

find_package(catkin REQUIRED COMPONENTS
  catkin_virtualenv
  message_generation
)

add_service_files(
  FILES
  Command.srv
  Status.srv
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

catkin_generate_virtualenv()

include_directories()

file(GLOB NODE_SCRIPTS_FILES node_scripts/*)
file(GLOB SCRIPTS_FILES scripts/*)
catkin_install_python(
  PROGRAMS ${NODE_SCRIPTS_FILES} ${SCRIPTS_FILES}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES requirements.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
