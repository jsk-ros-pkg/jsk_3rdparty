cmake_minimum_required(VERSION 2.8.3)
project(webrtcvad_ros)

find_package(catkin REQUIRED COMPONENTS catkin_virtualenv)

catkin_generate_virtualenv(
    PYTHON_INTERPRETER python3
    USE_SYSTEM_PACKAGES FALSE
    CHECK_VENV FALSE
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
    node_scripts/vad.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
