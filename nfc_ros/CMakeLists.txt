if("$ENV{ROS_VERSION}" EQUAL 2)
  cmake_minimum_required(VERSION 3.8)
  project(nfc_ros)

  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_python REQUIRED)
  find_package(rosidl_default_generators REQUIRED)

  # rosidl only allows upper case constant names, so the ROS2 Tag.msg is
  # generated from msg/Tag.msg with them upper cased
  # (NFC_TAG_FelicaStandard -> NFC_TAG_FELICASTANDARD, ...)
  file(READ msg/Tag.msg _tag_msg)
  string(REGEX MATCHALL "[A-Za-z0-9_]+=" _tag_constants "${_tag_msg}")
  foreach(_constant ${_tag_constants})
    string(TOUPPER "${_constant}" _upper_constant)
    string(REPLACE "${_constant}" "${_upper_constant}" _tag_msg "${_tag_msg}")
  endforeach()
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/ros2_msg/msg/Tag.msg "${_tag_msg}")
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${PROJECT_SOURCE_DIR}/msg/Tag.msg)

  rosidl_generate_interfaces(${PROJECT_NAME}
    msg/NDEFRecord.msg
    ${CMAKE_CURRENT_BINARY_DIR}/ros2_msg:msg/Tag.msg
  )

  # rosidl_generate_interfaces() already installs the nfc_ros python
  # package (with ament_python_install_package()), add the modules to it
  install(DIRECTORY python/${PROJECT_NAME}/
    DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
    PATTERN "__init__.py" EXCLUDE
  )

  # nfcpy is not available from apt, and catkin_virtualenv has no ROS2
  # port, so the node runs in a uv venv made from pyproject.toml/uv.lock
  # (same versions as requirements.txt). The installed ROS2 copy gets the
  # venv's shebang, keeping the source shared with ROS1.
  find_package(jsk_ros1_ros2_compat REQUIRED)
  jsk_ros1_ros2_compat_generate_uv_venv(CHECK_IMPORTS nfc)
  jsk_ros1_ros2_compat_install_python(PROGRAMS node_scripts/nfc_ros_node.py USE_VENV)

  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )

  ament_export_dependencies(rosidl_default_runtime)
  ament_package()

else() # if("$ENV{ROS_VERSION}" EQUAL 2)
cmake_minimum_required(VERSION 2.8.3)
project(nfc_ros)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    catkin_virtualenv
)

catkin_python_setup()

add_message_files(
  FILES
  NDEFRecord.msg
  Tag.msg
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime
)

if ("$ENV{ROS_DISTRO}" MATCHES "indigo" OR "$ENV{ROS_DISTRO}" MATCHES "kinetic")
    message(WARNING "nfc_ros requires python3.6 or newer. For indigo, virtualenv generation is skipped.")
else()
catkin_generate_virtualenv(
    PYTHON_INTERPRETER python3
    CHECK_VENV FALSE
)
file(GLOB PYTHON_SCRIPT_FILES node_scripts/*.py test/*.py)
catkin_install_python(
  PROGRAMS ${PYTHON_SCRIPT_FILES}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES requirements.txt
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING AND ("$ENV{ROS_DISTRO}" MATCHES "indigo" OR "$ENV{ROS_DISTRO}" MATCHES "kinetic"))
  find_package(rostest REQUIRED)
  add_rostest(test/test_rospy_node.test
    DEPENDENCIES ${PROJECT_NAME}_generate_virtualenv
  )
endif()
endif()

endif() # if("$ENV{ROS_VERSION}" EQUAL 2)
