cmake_minimum_required(VERSION 2.8.3)
project(voicevox)

find_package(catkin REQUIRED
  COMPONENTS
  catkin_virtualenv
)

set(INSTALL_DIR ${PROJECT_SOURCE_DIR})

catkin_package()

catkin_generate_virtualenv(
  INPUT_REQUIREMENTS requirements.in
  PYTHON_INTERPRETER python3
)

add_custom_command(
  OUTPUT voicevox_installed
  COMMAND make -f ${PROJECT_SOURCE_DIR}/Makefile
  MD5SUM_DIR=${PROJECT_SOURCE_DIR}/md5sum
  INSTALL_DIR=${INSTALL_DIR}
)


add_custom_command(
  OUTPUT voicevox_core_installed
  COMMAND make -f ${PROJECT_SOURCE_DIR}/Makefile.core
  MD5SUM_DIR=${PROJECT_SOURCE_DIR}/md5sum
  INSTALL_DIR=${INSTALL_DIR}
)

add_custom_target(all_installed ALL DEPENDS voicevox_installed voicevox_core_installed)

file(GLOB NODE_SCRIPTS_FILES node_scripts/*.py)
catkin_install_python(
  PROGRAMS ${NODE_SCRIPTS_FILES}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/node_scripts/
)
install(DIRECTORY node_scripts/voicevox_engine
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/catkin_virtualenv_scripts/
  USE_SOURCE_PERMISSIONS)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
install(PROGRAMS bin/text2wave
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/bin)

install(DIRECTORY
  ${INSTALL_DIR}/lib
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
