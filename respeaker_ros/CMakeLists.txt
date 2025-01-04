cmake_minimum_required(VERSION 2.8.3)
project(respeaker_ros)

find_package(catkin REQUIRED COMPONENTS
  catkin_virtualenv
  dynamic_reconfigure
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/Respeaker.cfg
)

catkin_package()

if("$ENV{ROS_DISTRO}" STREQUAL "kinetic" OR "$ENV{ROS_DISTRO}" STREQUAL "melodic")
  catkin_generate_virtualenv(
    PYTHON_INTERPRETER python2
    CHECK_VENV FALSE
    )
elseif("$ENV{ROS_DISTRO}" STREQUAL "indigo")
  file(DOWNLOAD http://curl.haxx.se/ca/cacert.pem ${CMAKE_BINARY_DIR}/cacert.pem)
  catkin_generate_virtualenv(
    PYTHON_INTERPRETER python2
    # https://stackoverflow.com/questions/25981703/pip-install-fails-with-connection-error-ssl-certificate-verify-failed
    EXTRA_PIP_ARGS -vvv --cert=${CMAKE_BINARY_DIR}/cacert.pem
  )
else()
  catkin_generate_virtualenv(
    PYTHON_INTERPRETER python3
    CHECK_VENV FALSE
    )
endif()

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
install(FILES requirements.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

file(GLOB PYTHON_SCRIPT_FILES scripts/*.py test/*.py)
catkin_install_python(PROGRAMS ${PYTHON_SCRIPT_FILES}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/sample_respeaker.test
    DEPENDENCIES ${PROJECT_NAME}_generate_virtualenv
  )
  add_rostest(test/test_rospy_node.test
    DEPENDENCIES ${PROJECT_NAME}_generate_virtualenv
  )
endif()
