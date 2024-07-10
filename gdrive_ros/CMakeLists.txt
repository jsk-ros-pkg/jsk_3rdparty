cmake_minimum_required(VERSION 2.8.3)
project(gdrive_ros)

find_package(catkin REQUIRED COMPONENTS
  catkin_virtualenv REQUIRED
  rospy
  message_generation
)

catkin_python_setup()

add_service_files(
  FILES
  Upload.srv
  MultipleUpload.srv
)

generate_messages(
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)

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


catkin_install_python(PROGRAMS node_scripts/gdrive_server_node.py node_scripts/sample_gdrive_rospy_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(DIRECTORY sample
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(DIRECTORY euslisp
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(FILES requirements.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/gdrive_server.launch)
endif()
