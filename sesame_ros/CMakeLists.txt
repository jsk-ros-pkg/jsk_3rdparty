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

if($ENV{ROS_DISTRO} STRGREATER "melodic")
  catkin_generate_virtualenv(
    PYTHON_INTERPRETER python3
    )
else()
  catkin_generate_virtualenv(
    PYTHON_INTERPRETER python2
    )
endif()

include_directories()

file(GLOB NODE_SCRIPTS_FILES node_scripts/*)
catkin_install_python(
  PROGRAMS ${NODE_SCRIPTS_FILES}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/node_scripts/)
file(GLOB SCRIPTS_FILES scripts/*)
catkin_install_python(
  PROGRAMS ${SCRIPTS_FILES}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts/)
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
