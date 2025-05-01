cmake_minimum_required(VERSION 2.8.3)
project(emotion_analyzer)

find_package(catkin REQUIRED COMPONENTS
  catkin_virtualenv
  std_msgs
  message_generation
  )

catkin_python_setup()

add_service_files(
  FILES
  AnalyzeText.srv
  AnalyzeAudio.srv
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  )

catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime
)

find_package(Python3 3.8 QUIET COMPONENTS Interpreter)
if(NOT Python3_FOUND)
  message(WARNING "emotion_analyzer (Hume AI) requires python3.8 or newer")
  return()
endif()
message(STATUS "Found Python: ${Python3_EXECUTABLE}")
message(STATUS "Python Version: ${Python3_VERSION}")

catkin_generate_virtualenv(
  INPUT_REQUIREMENTS requirements.in
  PYTHON_INTERPRETER python3
  CHECK_VENV FALSE
)

file(GLOB PYTHON_SCRIPTS scripts/*.py)
catkin_install_python(PROGRAMS ${PYTHON_SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
