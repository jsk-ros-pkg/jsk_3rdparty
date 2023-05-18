cmake_minimum_required(VERSION 2.8.3)
project(ros_google_cloud_language)

execute_process(COMMAND dpkg-architecture -qDEB_TARGET_GNU_CPU OUTPUT_VARIABLE ARCHITECTURE)
message(STATUS "DEB_TARGET_GNU_CPU : ${ARCHITECTURE}")
if(($ENV{ROS_DISTRO} STREQUAL "indigo") OR
   ($ENV{ROS_DISTRO} STREQUAL "kinetic" AND "${ARCHITECTURE}" MATCHES "i.*86" ))
  message(WARNING "following requirements.txt syntax is not support on $ENV{ROS_DISTRO}/${ARCHITECTURE}")
  message(WARNING "google-api-core[grpc]==1.31.5  # via google-cloud-language")
  find_package(catkin)
  catkin_package()
  return()
endif()

find_package(
  catkin REQUIRED COMPONENTS
  actionlib_msgs
  catkin_virtualenv
  diagnostic_msgs
)

add_message_files(
  FILES
  TextEntity.msg
  TextSyntax.msg
  PartOfSpeechTag.msg
  DependencyEdge.msg
)

add_action_files(
  FILES
  AnalyzeText.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  diagnostic_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
)

catkin_generate_virtualenv(
  # Disable creating a unit test to verify that package requirements are locked.
  # # commented out for python3, which uses concurrent.futures
  # # futures==3.3.0
  CHECK_VENV FALSE  # Default TRUE
  )

include_directories()

file(GLOB NODE_SCRIPTS_FILES node_scripts/*.py)
catkin_install_python(
  PROGRAMS ${NODE_SCRIPTS_FILES}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/node_scripts/)
file(GLOB PYTHON_TEST_SCRIPTS test/*.py)
catkin_install_python(PROGRAMS ${PYTHON_TEST_SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test/)

install(DIRECTORY launch samples
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(FILES requirements.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/test_rospy_node.test)
endif()
