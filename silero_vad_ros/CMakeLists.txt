cmake_minimum_required(VERSION 2.8.3)
project(silero_vad_ros)

find_program(EXIST_PYTHON3 "python3.8")
if(NOT EXIST_PYTHON3)
    message(WARNING "python3.8 command not found. exit without building")
    return()
else()
    message(STATUS "python3.8 command found. continue building...")
endif()

find_package(catkin REQUIRED COMPONENTS catkin_virtualenv)

catkin_generate_virtualenv(
    PYTHON_INTERPRETER python3.8
    USE_SYSTEM_PACKAGES FALSE
    CHECK_VENV FALSE
)

catkin_package(
)

catkin_install_python(PROGRAMS
    node_scripts/vad.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
