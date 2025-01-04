cmake_minimum_required(VERSION 2.8.3)
project(switchbot_ros)

find_package(
  catkin REQUIRED COMPONENTS
  message_generation
  actionlib_msgs
  catkin_virtualenv
)

catkin_python_setup()

add_message_files(
  FILES
  Device.msg
  DeviceArray.msg
  Meter.msg
  MeterProCO2.msg
  PlugMini.msg
  Hub2.msg
  Bot.msg
  StripLight.msg
)

add_action_files(
  DIRECTORY action
  FILES SwitchBotCommand.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
)

catkin_package()

if("$ENV{ROS_DISTRO}" STREQUAL "indigo")
file(DOWNLOAD http://curl.haxx.se/ca/cacert.pem ${CMAKE_BINARY_DIR}/cacert.pem)
catkin_generate_virtualenv(
  PYTHON_INTERPRETER python3
  CHECK_VENV FALSE
  # https://stackoverflow.com/questions/25981703/pip-install-fails-with-connection-error-ssl-certificate-verify-failed
  EXTRA_PIP_ARGS -vvv --cert=${CMAKE_BINARY_DIR}/cacert.pem
)
else()
catkin_generate_virtualenv(
  PYTHON_INTERPRETER python3
  CHECK_VENV FALSE
)
endif()

include_directories()

# install
# euslisp
file(GLOB EUSLISP_SCRIPTS scripts/*.l)
install(FILES ${EUSLISP_SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
# python
file(GLOB PYTHON_SCRIPTS scripts/*.py)
catkin_install_python(
  PROGRAMS ${PYTHON_SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
# launch
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

