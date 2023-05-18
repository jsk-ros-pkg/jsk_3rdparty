cmake_minimum_required(VERSION 2.8.3)
project(nfc_ros)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    catkin_virtualenv
)

catkin_python_setup()

add_message_files(
  FILES
  NDEFRecord.msg
  Tag.msg
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime
)

if ("$ENV{ROS_DISTRO}" MATCHES "indigo")
    message(WARNING "nfc_ros requires python3.6 or newer. For indigo, virtualenv generation is skipped.")
else()
catkin_generate_virtualenv(
    PYTHON_INTERPRETER python3
    CHECK_VENV FALSE
)
file(GLOB NODE_SCRIPTS_FILES node_scripts/*.py)
catkin_install_python(
  PROGRAMS ${NODE_SCRIPTS_FILES}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/node_scripts/)
file(GLOB PYTHON_TEST_SCRIPTS test/*.py)
catkin_install_python(PROGRAMS ${PYTHON_TEST_SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test/)

install(FILES requirements.txt
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/test_rospy_node.test
    DEPENDENCIES ${PROJECT_NAME}_generate_virtualenv
  )
endif()
endif()
