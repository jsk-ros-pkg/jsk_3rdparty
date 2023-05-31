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

catkin_package()

if($ENV{ROS_DISTRO} STRGREATER "melodic")
  catkin_generate_virtualenv(
    PYTHON_INTERPRETER python3
    )
else()
  catkin_generate_virtualenv(
    PYTHON_INTERPRETER python2
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
  find_package(catkin REQUIRED COMPONENTS rostest roslaunch)
  add_rostest(test/sample_ros_speech_recognition.test
    DEPENDENCIES ${PROJECT_NAME}_generate_virtualenv
  )
  roslaunch_add_file_check(launch/speech_recognition.launch)
  add_rostest(test/test_rospy_node.test
    DEPENDENCIES ${PROJECT_NAME}_generate_virtualenv
  )
endif()
