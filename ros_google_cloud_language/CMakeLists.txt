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
)

catkin_package(
  CATKIN_DEPENDS
)

# generate the virtualenv
find_package(Python3 QUIET COMPONENTS Interpreter)
if(Python3_FOUND)
  message(STATUS "Found Python: ${Python3_EXECUTABLE}")
  message(STATUS "Python Version: ${Python3_VERSION}")
endif()
if((NOT Python3_FOUND) OR (Python3_VERSION VERSION_LESS "3.12"))
catkin_generate_virtualenv(
  INPUT_REQUIREMENTS requirements.in
  # specify python version
  # https://github.com/jsk-ros-pkg/jsk_3rdparty/pull/367/files
  PYTHON_INTERPRETER "python$ENV{ROS_PYTHON_VERSION}"
  # Disable creating a unit test to verify that package requirements are locked.
  # # commented out for python3, which uses concurrent.futures
  # # futures==3.3.0
  CHECK_VENV FALSE  # Default TRUE
  )
else()
catkin_generate_virtualenv(
  INPUT_REQUIREMENTS requirements.in.python3.12
  PYTHON_INTERPRETER python3
  CHECK_VENV FALSE
  )
endif()
include_directories()

file(GLOB PYTHON_SCRIPT_FILES node_scripts/*.py test/*.py)
catkin_install_python(
  PROGRAMS ${PYTHON_SCRIPT_FILES}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch samples
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/test_rospy_node.test)
endif()
