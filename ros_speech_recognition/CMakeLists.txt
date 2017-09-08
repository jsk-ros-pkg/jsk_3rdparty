cmake_minimum_required(VERSION 2.8.3)
project(ros_speech_recognition)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/SpeechRecognition.cfg
)

catkin_package()

install(DIRECTORY scripts launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

