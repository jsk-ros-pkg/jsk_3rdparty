cmake_minimum_required(VERSION 2.8.3)
project(ros_speech_recognition)

find_package(catkin REQUIRED COMPONENTS
  catkin_virtualenv
  dynamic_reconfigure
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/SpeechRecognition.cfg
)

add_custom_target(${PROJECT_NAME}_install_trained_data ALL COMMAND python$ENV{ROS_PYTHON_VERSION} ${PROJECT_SOURCE_DIR}/scripts/install_trained_data.py)

catkin_package()

execute_process(COMMAND bash -c "gcc -dumpmachine" OUTPUT_VARIABLE gcc_dump_machine OUTPUT_STRIP_TRAILING_WHITESPACE)
message("-- gcc dumpmachine returns ${gcc_dump_machine}")
if($ENV{ROS_DISTRO} STREQUAL "indigo" OR
    (($ENV{ROS_DISTRO} STREQUAL "kinetic") AND (NOT(gcc_dump_machine MATCHES "x86_64-.*"))) OR
    (($ENV{ROS_DISTRO} STREQUAL "melodic") AND (NOT(gcc_dump_machine MATCHES "x86_64-.*"))))
  message(WARNING "SpeechRecognition==3.9.0 and vosk does not work on Python 3.4, which available on Indigo")
  message(WARNING "SpeechRecognition==3.9.0 and vosk does not work on Arm nor i386")
  message(WARNING "so we only use old SpeechRecognition without vosk")
  file(COPY requirements.txt.indigo DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
  execute_process(
    COMMAND cp ${CMAKE_CURRENT_BINARY_DIR}/requirements.txt.indigo ${CMAKE_CURRENT_SOURCE_DIR}/requirements.txt
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
  if("$ENV{ROS_DISTRO}" STREQUAL "indigo")
    file(DOWNLOAD http://curl.haxx.se/ca/cacert.pem ${CMAKE_BINARY_DIR}/cacert.pem)
    catkin_generate_virtualenv(
      PYTHON_INTERPRETER python2
      # https://stackoverflow.com/questions/25981703/pip-install-fails-with-connection-error-ssl-certificate-verify-failed
      EXTRA_PIP_ARGS -vvv --cert=${CMAKE_BINARY_DIR}/cacert.pem
    )
  else()
    catkin_generate_virtualenv(PYTHON_INTERPRETER python2)
  endif()
else()
catkin_generate_virtualenv(
  PYTHON_INTERPRETER python3
  CHECK_VENV FALSE
    )
endif()

file(GLOB PYTHON_SCRIPT_FILES scripts/*.py test/*.py)
catkin_install_python(
  PROGRAMS ${PYTHON_SCRIPT_FILES}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES requirements.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(roslaunch REQUIRED)
  add_rostest(test/sample_ros_speech_recognition.test
    DEPENDENCIES ${PROJECT_NAME}_generate_virtualenv
  )
  roslaunch_add_file_check(launch/speech_recognition.launch)
  add_rostest(test/test_rospy_node.test
    DEPENDENCIES ${PROJECT_NAME}_generate_virtualenv
  )
endif()
