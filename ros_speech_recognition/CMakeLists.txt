if("$ENV{ROS_VERSION}" EQUAL 2)
  cmake_minimum_required(VERSION 3.8)
  project(ros_speech_recognition)

  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_python REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})

  # dynamic_reconfigure has no ROS2 port: its parameters are ROS2
  # parameters, see cfg/ros2/SpeechRecognitionConfig.py
  install(DIRECTORY cfg/ros2/
    DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}/cfg
  )

  # SpeechRecognition/vosk are not available from apt, and catkin_virtualenv
  # has no ROS2 port, so the recognition node runs in a uv venv made from
  # pyproject.toml/uv.lock (same versions as requirements.txt). The installed
  # ROS2 copies get a python3 (or the venv's) shebang, keeping the source
  # shared with ROS1.
  find_package(jsk_ros1_ros2_compat REQUIRED)
  jsk_ros1_ros2_compat_generate_uv_venv(CHECK_IMPORTS speech_recognition vosk)
  jsk_ros1_ros2_compat_install_python(PROGRAMS scripts/speech_recognition_node.py USE_VENV)
  jsk_ros1_ros2_compat_install_python(PROGRAMS scripts/parrot_node.py scripts/speech_recognition_candidates_to_string.py)

  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )

  # download the vosk models of recognize_vosk.py with jsk_data, as the ROS1
  # ${PROJECT_NAME}_install_trained_data target does, into the installed
  # share directory: at install time, by an absolute path, as ament_index
  # does not find this package while it is being built
  install(CODE "execute_process(COMMAND ${Python3_EXECUTABLE} ${PROJECT_SOURCE_DIR}/scripts/install_trained_data.py --data-dir \$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/trained_data)")

  ament_package()

else() # if("$ENV{ROS_VERSION}" EQUAL 2)
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

endif() # if("$ENV{ROS_VERSION}" EQUAL 2)
