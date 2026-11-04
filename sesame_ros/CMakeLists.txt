if("$ENV{ROS_VERSION}" EQUAL 2)
  cmake_minimum_required(VERSION 3.8)
  project(sesame_ros)

  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)

  rosidl_generate_interfaces(${PROJECT_NAME}
    srv/Command.srv
    srv/Status.srv
  )

  # ROS2 uses the system python3-requests instead of catkin_virtualenv.
  # The installed ROS2 copies get a python3 shebang (ROS2 base images ship
  # no bare `python`), keeping the source shared with ROS1.
  find_package(jsk_ros1_ros2_compat REQUIRED)
  jsk_ros1_ros2_compat_install_python(PROGRAMS node_scripts/sesame_server.py scripts/list_sesames.py)

  ament_package()

else() # if("$ENV{ROS_VERSION}" EQUAL 2)
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

if($ENV{ROS_DISTRO} STREQUAL "kinetic" OR $ENV{ROS_DISTRO} STREQUAL "melodic")
  catkin_generate_virtualenv(
    INPUT_REQUIREMENTS requirements.in.python2
    PYTHON_INTERPRETER python2
    CHECK_VENV FALSE
    )
elseif("$ENV{ROS_DISTRO}" STREQUAL "indigo")
  file(DOWNLOAD http://curl.haxx.se/ca/cacert.pem ${CMAKE_BINARY_DIR}/cacert.pem)
  catkin_generate_virtualenv(
    INPUT_REQUIREMENTS requirements.in.python2
    PYTHON_INTERPRETER python2
    # https://stackoverflow.com/questions/25981703/pip-install-fails-with-connection-error-ssl-certificate-verify-failed
    EXTRA_PIP_ARGS -vvv --cert=${CMAKE_BINARY_DIR}/cacert.pem
  )
else()
find_package(Python3 QUIET COMPONENTS Interpreter)
if(Python3_FOUND)
  message(STATUS "Found Python: ${Python3_EXECUTABLE}")
  message(STATUS "Python Version: ${Python3_VERSION}")
endif()
if((NOT Python3_FOUND) OR (Python3_VERSION VERSION_LESS "3.10"))
  catkin_generate_virtualenv(
    INPUT_REQUIREMENTS requirements.in.python2
    PYTHON_INTERPRETER python3
    CHECK_VENV FALSE
    )
else()
  catkin_generate_virtualenv(
    INPUT_REQUIREMENTS requirements.in.python3.12
    PYTHON_INTERPRETER python3
  )
endif()
endif()

include_directories()

file(GLOB PYTHON_SCRIPT_FILES node_scripts/* scripts/* test/*.py)
catkin_install_python(
  PROGRAMS ${PYTHON_SCRIPT_FILES}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/test_rospy_node.test
    DEPENDENCIES ${PROJECT_NAME}_generate_virtualenv
  )
endif()

endif() # if("$ENV{ROS_VERSION}" EQUAL 2)
