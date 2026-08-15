cmake_minimum_required(VERSION 2.8.3)
project(smart_device_protocol)

find_package(catkin REQUIRED COMPONENTS message_generation catkin_virtualenv std_msgs)

catkin_generate_virtualenv(
  PYTHON_INTERPRETER python3
  CHECK_VENV FALSE 
  USE_SYSTEM_PACKAGES FALSE
  ISOLATE_REQUIREMENTS TRUE
)

catkin_python_setup()

add_message_files(
  FILES
  Packet.msg
  UWBDistance.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime
)

file(GLOB ${PROJECT_NAME}_node_scripts ${PROJECT_SOURCE_DIR}/node_scripts/*)

catkin_install_python(
  PROGRAMS ${${PROJECT_NAME}_node_scripts}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  file(GLOB ${PROJECT_NAME}_test_node_scripts ${PROJECT_SOURCE_DIR}/tests/*.py)
  catkin_install_python(
    PROGRAMS ${${PROJECT_NAME}_test_node_scripts}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  add_rostest(tests/sdp_v2_interface_rostest.test)
endif()
