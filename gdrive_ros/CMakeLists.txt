if("$ENV{ROS_VERSION}" EQUAL 2)
  cmake_minimum_required(VERSION 3.8)
  project(gdrive_ros)

  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_python REQUIRED)
  find_package(rosidl_default_generators REQUIRED)

  rosidl_generate_interfaces(${PROJECT_NAME}
    srv/Upload.srv
    srv/MultipleUpload.srv
  )

  # rosidl_generate_interfaces() already installs the gdrive_ros python
  # package (gdrive_ros.srv), so add gdrive_ros_client.py into it rather
  # than calling ament_python_install_package() again.
  install(FILES src/${PROJECT_NAME}/gdrive_ros_client.py
    DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
  )

  # PyDrive is not available from apt, and catkin_virtualenv has no ROS2
  # port, so the server runs in a uv venv made from pyproject.toml/uv.lock.
  # The installed ROS2 copies get a python3 (or the venv's) shebang,
  # keeping the source shared with ROS1.
  find_package(jsk_ros1_ros2_compat REQUIRED)
  jsk_ros1_ros2_compat_generate_uv_venv(CHECK_IMPORTS pydrive)
  jsk_ros1_ros2_compat_install_python(PROGRAMS node_scripts/gdrive_server_node.py USE_VENV)
  jsk_ros1_ros2_compat_install_python(PROGRAMS node_scripts/sample_gdrive_rospy_client.py)

  install(DIRECTORY launch sample euslisp
    DESTINATION share/${PROJECT_NAME}
    USE_SOURCE_PERMISSIONS
  )

  ament_package()

else() # if("$ENV{ROS_VERSION}" EQUAL 2)
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

endif() # if("$ENV{ROS_VERSION}" EQUAL 2)
