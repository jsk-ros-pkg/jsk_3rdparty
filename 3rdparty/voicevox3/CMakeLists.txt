cmake_minimum_required(VERSION 2.8.3)
project(voicevox3)

find_package(catkin REQUIRED COMPONENTS catkin_virtualenv)

catkin_python_setup()

catkin_package()

catkin_generate_virtualenv(
  INPUT_REQUIREMENTS requirements.txt
  PYTHON_INTERPRETER python3
  USE_SYSTEM_PACKAGES FALSE
  CHECK_VENV FALSE  # Default TRUE
  )

include(ExternalProject)
ExternalProject_Add(voicevox_engine
  GIT_REPOSITORY    https://github.com/VOICEVOX/voicevox_engine
  GIT_TAG           0.14.7   # latest version before Python 3.11, Oct 5, 2023
  CONFIGURE_COMMAND ""
  BUILD_COMMAND     ${CMAKE_COMMAND} -E remove_directory ${PROJECT_SOURCE_DIR}/voicevox_engine/speaker_info.orig
  INSTALL_COMMAND   ${CMAKE_COMMAND} -E rename ${PROJECT_SOURCE_DIR}/voicevox_engine/speaker_info ${PROJECT_SOURCE_DIR}/voicevox_engine/speaker_info.orig
  SOURCE_DIR  ${PROJECT_SOURCE_DIR}/voicevox_engine
  )
ExternalProject_Add(voicevox_resource
  GIT_REPOSITORY    https://github.com/VOICEVOX/voicevox_resource
  GIT_TAG           main
  GIT_PROGRESS      TRUE
  GIT_SHALLOW       TRUE
  BUILD_IN_SOURCE   TRUE
  CONFIGURE_COMMAND ""
  BUILD_COMMAND     ""
  INSTALL_COMMAND   python3 ./scripts/clean_character_info.py --output_dir ${PROJECT_SOURCE_DIR}/voicevox_engine/speaker_info
  DEPENDS voicevox_engine
  )
#set(CORE_VERSION 0.15.7) ## Dec 29, 2024
#set(CORE_HASH "444dc362d98e065b8581e5a9e403b8fc")
set(CORE_VERSION 0.14.6) ## Jan 11, 2024
set(CORE_HASH "26719dab23a8e0b4559516d1f2a78833")
ExternalProject_Add(voicevox_core
  URL               https://github.com/VOICEVOX/voicevox_core/releases/download/${CORE_VERSION}/voicevox_core-linux-x64-cpu-${CORE_VERSION}.zip
  URL_HASH          MD5=${CORE_HASH}
  CONFIGURE_COMMAND ""
  BUILD_COMMAND     ""
  INSTALL_COMMAND   ""
  SOURCE_DIR  ${PROJECT_SOURCE_DIR}/voicevox_core
)

catkin_install_python(
  PROGRAMS node_scripts/request_synthesis.py node_scripts/list_speakers.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/node_scripts/)
install(
  PROGRAMS bin/text2wave
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/bin)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

install(DIRECTORY
  voicevox_engine voicevox_core
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
