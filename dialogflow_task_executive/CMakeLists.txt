cmake_minimum_required(VERSION 2.8.3)
project(dialogflow_task_executive)

if($ENV{ROS_DISTRO} STREQUAL "indigo")
  message(WARNING "following requirements.txt syntax is not support on 14.04")
  message(WARNING "google-api-core[grpc]==1.31.5  # via google-cloud-language")
  find_package(catkin)
  catkin_package()
  return()
endif()

find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
    catkin_virtualenv
)

add_message_files(
  FILES
  DialogResponse.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

catkin_generate_virtualenv(INPUT_REQUIREMENTS requirements.in)

file(GLOB NODE_SCRIPTS_FILES node_scripts/*.py)
catkin_install_python(
  PROGRAMS ${NODE_SCRIPTS_FILES}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
