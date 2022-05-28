cmake_minimum_required(VERSION 2.8.3)
project(voicevox)

find_package(catkin REQUIRED
  COMPONENTS
  catkin_virtualenv
)

catkin_python_setup()

set(INSTALL_DIR ${PROJECT_SOURCE_DIR})

catkin_package()

catkin_generate_virtualenv(
  INPUT_REQUIREMENTS requirements.in
  PYTHON_INTERPRETER python3
  USE_SYSTEM_PACKAGES FALSE
)

add_custom_command(
  OUTPUT voicevox_model_installed
  COMMAND make -f ${PROJECT_SOURCE_DIR}/Makefile.model
  MD5SUM_DIR=${PROJECT_SOURCE_DIR}/md5sum
  INSTALL_DIR=${INSTALL_DIR}
)


add_custom_command(
  OUTPUT voicevox_core_installed
  COMMAND make -f ${PROJECT_SOURCE_DIR}/Makefile.core
  MD5SUM_DIR=${PROJECT_SOURCE_DIR}/md5sum
  INSTALL_DIR=${INSTALL_DIR}
)

add_custom_command(
  OUTPUT voicevox_engine_installed
  COMMAND make -f ${PROJECT_SOURCE_DIR}/Makefile.engine
  MD5SUM_DIR=${PROJECT_SOURCE_DIR}/md5sum
  INSTALL_DIR=${INSTALL_DIR}
)

add_custom_command(
  OUTPUT open_jtalk_dic_installed
  COMMAND make -f ${PROJECT_SOURCE_DIR}/Makefile.open_jtalk_dic
  MD5SUM_DIR=${PROJECT_SOURCE_DIR}/md5sum
  INSTALL_DIR=${INSTALL_DIR}
)

add_custom_target(all_installed ALL DEPENDS
  voicevox_model_installed
  voicevox_core_installed
  voicevox_engine_installed
  open_jtalk_dic_installed)

file(GLOB NODE_SCRIPTS_FILES node_scripts/*.py)
catkin_install_python(
  PROGRAMS ${NODE_SCRIPTS_FILES}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/node_scripts/
)
install(DIRECTORY node_scripts/voicevox_engine
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/catkin_virtualenv_scripts/
  USE_SOURCE_PERMISSIONS)
install(DIRECTORY launch dict
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
install(PROGRAMS bin/text2wave
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/bin)

install(DIRECTORY
  ${INSTALL_DIR}/lib
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
