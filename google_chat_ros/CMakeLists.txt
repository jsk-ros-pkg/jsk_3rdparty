if("$ENV{ROS_VERSION}" EQUAL 2)
  cmake_minimum_required(VERSION 3.8)
  project(google_chat_ros)

  # NOTE: the euslisp client stays ROS1-only.
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_python REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(std_msgs REQUIRED)

  file(GLOB _msg_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} msg/*.msg)
  rosidl_generate_interfaces(${PROJECT_NAME}
    ${_msg_files}
    action/SendMessage.action
    DEPENDENCIES std_msgs
  )

  # rosidl_generate_interfaces() already installs the generated python
  # package `google_chat_ros` with ament_python_install_package(), so
  # merge src/google_chat_ros/ into it instead of a second one.
  install(DIRECTORY src/${PROJECT_NAME}/
    DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
    PATTERN "*.pyc" EXCLUDE
  )

  # The python dependencies (google api, pubsub, gdown, ...) are not all
  # available from apt, and catkin_virtualenv has no ROS2 port, so the node
  # runs in a uv venv made from pyproject.toml/uv.lock (same versions as
  # requirements.in.python3.12). helper.py needs none of them.
  find_package(jsk_ros1_ros2_compat REQUIRED)
  jsk_ros1_ros2_compat_generate_uv_venv(
    CHECK_IMPORTS gdown googleapiclient google.cloud.pubsub_v1 oauth2client.client)
  jsk_ros1_ros2_compat_install_python(PROGRAMS scripts/google_chat_ros_node.py USE_VENV)
  jsk_ros1_ros2_compat_install_python(PROGRAMS scripts/helper.py)

  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.launch.xml" PATTERN "*.yaml"
  )

  ament_export_dependencies(rosidl_default_runtime)
  ament_package()

else() # if("$ENV{ROS_VERSION}" EQUAL 2)
cmake_minimum_required(VERSION 2.8.3)
project(google_chat_ros)

find_package(
  catkin REQUIRED COMPONENTS
  catkin_virtualenv
  actionlib_msgs
  std_msgs
  message_generation
  )

catkin_python_setup()

add_message_files(
  DIRECTORY msg
  )

add_action_files(
  FILES
  SendMessage.action
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
  )

catkin_package(
  CATKIN_DEPENDS message_runtime
  )

# generate the virtualenv
find_package(Python3 QUIET COMPONENTS Interpreter)
if(Python3_FOUND)
  message(STATUS "Found Python: ${Python3_EXECUTABLE}")
  message(STATUS "Python Version: ${Python3_VERSION}")
endif()
if((NOT Python3_FOUND) OR (Python3_VERSION VERSION_LESS "3.12"))
catkin_generate_virtualenv(
  INPUT_REQUIREMENTS requirements.in
  PYTHON_INTERPRETER python3
  USE_SYSTEM_PACKAGES FALSE
  CHECK_VENV FALSE
  )
else()
catkin_generate_virtualenv(
  INPUT_REQUIREMENTS requirements.in.python3.12
  PYTHON_INTERPRETER python3
  USE_SYSTEM_PACKAGES FALSE
  CHECK_VENV FALSE
  )
endif()

# install
# euslisp
file(GLOB EUSLISP_SCRIPTS scripts/*.l)
install(FILES ${EUSLISP_SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts/)

# python
file(GLOB PYTHON_SCRIPT_FILES scripts/*.py  test/*.py)
catkin_install_python(
  PROGRAMS ${PYTHON_SCRIPT_FILES}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# launch
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

# import test
if(CATKIN_ENABLE_TESTING)
  find_package(catkin_virtualenv REQUIRED)
  find_package(rostest REQUIRED)
  catkin_install_python(
    PROGRAMS test/test_import.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
  add_rostest(test/import.test
    DEPENDENCIES ${PROJECT_NAME}_generate_virtualenv
    )
  add_rostest(test/test_rospy_node.test
    DEPENDENCIES ${PROJECT_NAME}_generate_virtualenv
  )
endif()

endif() # if("$ENV{ROS_VERSION}" EQUAL 2)
