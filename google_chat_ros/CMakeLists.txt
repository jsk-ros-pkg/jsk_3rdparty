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
