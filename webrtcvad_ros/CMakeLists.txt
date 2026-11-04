cmake_minimum_required(VERSION 2.8.3)
project(webrtcvad_ros)

find_package(catkin REQUIRED)

catkin_package()

# python3 only; python2 environments install webrtcvad with pip
if("$ENV{ROS_PYTHON_VERSION}" STREQUAL "3")
  find_package(catkin_virtualenv REQUIRED)
  catkin_generate_virtualenv(
    PYTHON_INTERPRETER python3
    CHECK_VENV FALSE
  )
  file(GLOB PYTHON_SCRIPT_FILES node_scripts/*.py)
  catkin_install_python(
    PROGRAMS ${PYTHON_SCRIPT_FILES}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    USE_SOURCE_PERMISSIONS)
  install(FILES requirements.txt
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endif()
