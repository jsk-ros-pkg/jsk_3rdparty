if("$ENV{ROS_VERSION}" EQUAL 2)
  cmake_minimum_required(VERSION 3.8)
  project(switchbot_ros)

  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_python REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(std_msgs REQUIRED)

  rosidl_generate_interfaces(${PROJECT_NAME}
    msg/Device.msg
    msg/DeviceArray.msg
    msg/Meter.msg
    msg/MeterProCO2.msg
    msg/PlugMini.msg
    msg/Hub2.msg
    msg/Bot.msg
    msg/StripLight.msg
    action/SwitchBotCommand.action
    DEPENDENCIES std_msgs
  )

  # rosidl_generate_interfaces() already installs the python package
  # ${PROJECT_NAME} (msg/action), so put switchbot.py and
  # switchbot_ros_client.py next to it instead of
  # ament_python_install_package().
  install(FILES src/${PROJECT_NAME}/switchbot.py src/${PROJECT_NAME}/switchbot_ros_client.py
    DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
  )

  # The installed ROS2 copies get a python3 shebang (ROS2 base images ship
  # no bare `python`), keeping the source shared with ROS1.
  # switchbot.l (roseus client) is ROS1 only.
  find_package(jsk_ros1_ros2_compat REQUIRED)
  jsk_ros1_ros2_compat_install_python(PROGRAMS scripts/switchbot_ros_server.py scripts/switchbot_status_publisher.py scripts/control_switchbot.py)

  install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )

  ament_package()

else() # if("$ENV{ROS_VERSION}" EQUAL 2)
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

endif() # if("$ENV{ROS_VERSION}" EQUAL 2)
