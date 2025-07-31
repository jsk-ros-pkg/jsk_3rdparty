cmake_minimum_required(VERSION 2.8.3)
project(ros_translator)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure catkin_virtualenv)

if(NOT (("$ENV{ROS_DISTRO}" STREQUAL "noetic") OR
        ("$ENV{ROS_DISTRO}" STREQUAL "melodic")))
    message(WARNINIG "ros_translator can only be built for melodic and noetic")
    return()
endif()

catkin_python_setup()

catkin_generate_virtualenv(
    PYTHON_INTERPRETER python3
    USE_SYSTEM_PACKAGE FALSE
    CHECK_VENV FALSE
)


generate_dynamic_reconfigure_options(
    cfg/ROSTranslator.cfg
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/ros_translator_node.py
  scripts/demo_input_node.py
  scripts/demo_output_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES requirements.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
