cmake_minimum_required(VERSION 3.0.2)
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
