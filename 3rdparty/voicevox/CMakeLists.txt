# Fall back to ROS1 when $ROS_VERSION is unset (pre-ros_environment distros,
# e.g. indigo); every ROS2 environment always sets it via ros_environment.
if("$ENV{ROS_VERSION}" STREQUAL "")
  set(ENV{ROS_VERSION} 1)
endif()

if("$ENV{ROS_VERSION}" EQUAL 1)
cmake_minimum_required(VERSION 2.8.3)
else()
cmake_minimum_required(VERSION 3.19)
endif()
project(voicevox)

if("$ENV{ROS_VERSION}" EQUAL 1)
find_package(catkin REQUIRED COMPONENTS catkin_virtualenv)

catkin_package()
else()
find_package(ament_cmake REQUIRED)
endif()

execute_process(COMMAND bash -c "gcc -dumpmachine" OUTPUT_VARIABLE gcc_dump_machine OUTPUT_STRIP_TRAILING_WHITESPACE)
message("-- gcc dumpmachine returns ${gcc_dump_machine}")
if(gcc_dump_machine MATCHES "x86_64-.*")
 set(ARCH x64)
elseif(gcc_dump_machine MATCHES "aarch64-.*")
 set(ARCH arm64)
else()
  message(WARNING "unsupported platform")
  return()
endif()

find_package(Python3 QUIET COMPONENTS Interpreter)
if(NOT Python3_FOUND)
  message(WARNING "voicevox requires python3")
  return()
endif()
message(STATUS "Found Python: ${Python3_EXECUTABLE}")
message(STATUS "Python Version: ${Python3_VERSION}")

if(Python3_VERSION VERSION_LESS "3.12")
  set(REQUIREMENTS_FILE requirements_python3.8.txt)

  set(CORE_VERSION 0.14.6) ## Jan 11, 2024
  set(CORE_HASH_x64   "26719dab23a8e0b4559516d1f2a78833")
  set(CORE_HASH_arm64 "5a94142e04c43624ae44cf926ca42c29")
  set(ENGINE_VERSION 0.14.7)   # latest version before Python 3.11, Oct 5, 2023

else()
  set(REQUIREMENTS_FILE requirements_python3.12.txt)

  set(CORE_VERSION 0.15.7) ## Dec 29, 2024
  set(CORE_HASH_x64   "444dc362d98e065b8581e5a9e403b8fc")
  set(CORE_HASH_arm64 "fd33e2bf200119f70dece4b6ab58b3b0")
  set(ENGINE_VERSION 0.23.0)   # Mar 2, 2025

endif()

if("$ENV{ROS_VERSION}" EQUAL 1)
catkin_generate_virtualenv(
  INPUT_REQUIREMENTS ${REQUIREMENTS_FILE}
  PYTHON_INTERPRETER python3
  USE_SYSTEM_PACKAGES FALSE
  CHECK_VENV FALSE  # Default TRUE
  )
# catkin_virtualenv (>= 0.19) preinstalls setuptools 82 in the venv for
# python >= 3.12, which has no pkg_resources any more (pyworld imports it),
# and pip-compile drops the setuptools pin of ${REQUIREMENTS_FILE}, so put
# back setuptools < 81 in the devel/install venvs it generates
if(NOT Python3_VERSION VERSION_LESS "3.12")
  add_custom_command(TARGET ${PROJECT_NAME}_generate_virtualenv POST_BUILD
    COMMAND ${${PROJECT_NAME}_VENV_DEVEL_DIR}/bin/python -m pip install -qq "setuptools<81"
    COMMAND ${CMAKE_BINARY_DIR}/install/venv/bin/python -m pip install -qq "setuptools<81"
    COMMENT "Install setuptools<81 (with pkg_resources) to the virtualenvs"
    VERBATIM)
endif()

set(SHARE_DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
set(VOICEVOX_TARGET_DIR ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION})
else()
# ROS2: create virtualenv from pyproject.toml/uv.lock (catkin_virtualenv has
# no ROS2 port), installed to share/voicevox/venv
find_package(jsk_ros1_ros2_compat REQUIRED)
jsk_ros1_ros2_compat_generate_uv_venv(PYTHON_INTERPRETER ${Python3_EXECUTABLE} STRIP)
# pyopenjtalk downloads its dictionary on first use, which the installed
# (read-only) venv can not do, so do it at build time
execute_process(COMMAND ${voicevox_VENV_DIR}/bin/python3 -c "import pyopenjtalk; pyopenjtalk.g2p('a')"
  COMMAND_ERROR_IS_FATAL ANY)

set(SHARE_DESTINATION share/${PROJECT_NAME})
set(VOICEVOX_TARGET_DIR ${CMAKE_CURRENT_BINARY_DIR}/voicevox)
endif()
include(ExternalProject)
ExternalProject_Add(voicevox_engine
  GIT_REPOSITORY    https://github.com/VOICEVOX/voicevox_engine
  GIT_TAG           ${ENGINE_VERSION}
  BUILD_IN_SOURCE   TRUE
  CONFIGURE_COMMAND ""
  BUILD_COMMAND
    COMMAND test -e speaker_info && mv speaker_info speaker_info.orig || echo "OK"
    COMMAND test -e resources/character_info && mv resources/character_info resources/character_info.orig || echo "OK"
  INSTALL_COMMAND   ${CMAKE_COMMAND} -E copy_directory ./ ${VOICEVOX_TARGET_DIR}/voicevox_engine
  )
ExternalProject_Add(voicevox_resource
  GIT_REPOSITORY    https://github.com/VOICEVOX/voicevox_resource
  GIT_TAG           main
  GIT_PROGRESS      TRUE
  GIT_SHALLOW       TRUE
  BUILD_IN_SOURCE   TRUE
  CONFIGURE_COMMAND ""
  BUILD_COMMAND     ""
  INSTALL_COMMAND
    COMMAND test -e ${VOICEVOX_TARGET_DIR}/voicevox_engine/speaker_info || python3 ./scripts/clean_character_info.py --output_dir ${VOICEVOX_TARGET_DIR}/voicevox_engine/speaker_info
    COMMAND test -e ${VOICEVOX_TARGET_DIR}/voicevox_engine/resources/character_info || python3 ./scripts/clean_character_info.py --output_dir ${VOICEVOX_TARGET_DIR}/voicevox_engine/resources/character_info
  DEPENDS voicevox_engine
  )
ExternalProject_Add(voicevox_core
  URL               https://github.com/VOICEVOX/voicevox_core/releases/download/${CORE_VERSION}/voicevox_core-linux-${ARCH}-cpu-${CORE_VERSION}.zip
  URL_HASH          MD5=${CORE_HASH_${ARCH}}
  BUILD_IN_SOURCE   TRUE
  CONFIGURE_COMMAND ""
  BUILD_COMMAND     ""
  INSTALL_COMMAND   ${CMAKE_COMMAND} -E copy_directory ./ ${VOICEVOX_TARGET_DIR}/voicevox_core
  )

if("$ENV{ROS_VERSION}" EQUAL 1)
# deb build environment set CATKIN_BUILD_BINARY_PACKAGE
if("${voicevox_INSTALL_PREFIX}" OR "${CATKIN_BUILD_BINARY_PACKAGE}")
  set(ENGINE_PATH ${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION})
  set(VENV_PATH ${${PROJECT_NAME}_VENV_INSTALL_DIR})
else()
  set(ENGINE_PATH ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION})
  set(VENV_PATH ${${PROJECT_NAME}_VENV_DEVEL_DIR})
endif()
else()
set(ENGINE_PATH ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME})
set(VENV_PATH ${voicevox_VENV_INSTALL_DIR})
endif()
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/bin/run-voicevox.in
  ${VOICEVOX_TARGET_DIR}/bin/run-voicevox
  @ONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/bin/run-voicevox-docker.in
  ${VOICEVOX_TARGET_DIR}/bin/run-voicevox-docker
  @ONLY)

if("$ENV{ROS_VERSION}" EQUAL 1)
catkin_install_python(
  PROGRAMS node_scripts/request_synthesis.py node_scripts/list_speakers.py node_scripts/text_to_sound_request.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/node_scripts/)
else()
# `ros2 run voicevox <script>` runs executables in lib/voicevox
jsk_ros1_ros2_compat_install_python(
  PROGRAMS node_scripts/request_synthesis.py node_scripts/list_speakers.py node_scripts/text_to_sound_request.py
  USE_VENV)
install(PROGRAMS
  ${VOICEVOX_TARGET_DIR}/bin/run-voicevox ${VOICEVOX_TARGET_DIR}/bin/run-voicevox-docker
  DESTINATION lib/${PROJECT_NAME})
endif()

install(
  PROGRAMS bin/text2wave ${VOICEVOX_TARGET_DIR}/bin/run-voicevox ${VOICEVOX_TARGET_DIR}/bin/run-voicevox-docker
  DESTINATION ${SHARE_DESTINATION}/bin)

install(DIRECTORY launch
  DESTINATION ${SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

install(DIRECTORY
  ${VOICEVOX_TARGET_DIR}/voicevox_engine ${VOICEVOX_TARGET_DIR}/voicevox_core
  DESTINATION ${SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

if("$ENV{ROS_VERSION}" EQUAL 2)
ament_package()
endif()
