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

if ("$ENV{ROS_DISTRO}" MATCHES "indigo" OR "$ENV{ROS_DISTRO}" MATCHES "kinetic")
    message(WARNING "nfc_ros requires python3.6 or newer. For indigo, virtualenv generation is skipped.")
else()
catkin_generate_virtualenv(
    PYTHON_INTERPRETER python3
    CHECK_VENV FALSE
)
file(GLOB PYTHON_SCRIPT_FILES node_scripts/*.py test/*.py)
catkin_install_python(
  PROGRAMS ${PYTHON_SCRIPT_FILES}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES requirements.txt
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING AND ("$ENV{ROS_DISTRO}" MATCHES "indigo" OR "$ENV{ROS_DISTRO}" MATCHES "kinetic"))
  find_package(rostest REQUIRED)
  add_rostest(test/test_rospy_node.test
    DEPENDENCIES ${PROJECT_NAME}_generate_virtualenv
  )
endif()
endif()
