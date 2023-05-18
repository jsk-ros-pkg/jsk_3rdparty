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

if($ENV{ROS_DISTRO} STRGREATER "melodic")
  catkin_generate_virtualenv(
    PYTHON_INTERPRETER python3
    )
else()
  catkin_generate_virtualenv(
    PYTHON_INTERPRETER python2
    )
endif()

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
install(FILES requirements.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

file(GLOB PYTHON_SCRIPTS scripts/*.py)
catkin_install_python(PROGRAMS ${PYTHON_SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts/)
file(GLOB PYTHON_TEST_SCRIPTS test/*.py)
catkin_install_python(PROGRAMS ${PYTHON_TEST_SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test/)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/sample_respeaker.test
    DEPENDENCIES ${PROJECT_NAME}_generate_virtualenv
  )
  add_rostest(test/test_rospy_node.test
    DEPENDENCIES ${PROJECT_NAME}_generate_virtualenv
  )
endif()
