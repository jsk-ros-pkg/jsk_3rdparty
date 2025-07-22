cmake_minimum_required(VERSION 2.8.3)
project(julius)

find_package(catkin REQUIRED)

catkin_package()

set(MD5SUM_DIR ${PROJECT_SOURCE_DIR}/md5sum)
set(PATCH_DIR ${PROJECT_SOURCE_DIR}/patch)
set(INSTALL_DIR ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION})

add_custom_command(
  OUTPUT julius_installed
  COMMAND make -f ${PROJECT_SOURCE_DIR}/Makefile.julius MD5SUM_DIR=${MD5SUM_DIR} PATCH_DIR=${PATCH_DIR} INSTALL_DIR=${INSTALL_DIR}
)

add_custom_command(
  OUTPUT grammar_kit_installed
  COMMAND make -f ${PROJECT_SOURCE_DIR}/Makefile.grammar-kit MD5SUM_DIR=${MD5SUM_DIR} PATCH_DIR=${PATCH_DIR} INSTALL_DIR=${INSTALL_DIR}
)

add_custom_command(
  OUTPUT dictation_kit_installed
  COMMAND make -f ${PROJECT_SOURCE_DIR}/Makefile.dictation-kit MD5SUM_DIR=${MD5SUM_DIR} PATCH_DIR=${PATCH_DIR} INSTALL_DIR=${INSTALL_DIR}
)

# on ROS-O do not download grammer_kit, dictation_kit, instead install download script
if(NOT $ENV{ROS_DISTRO} STREQUAL "debian")
add_custom_target(all_installed ALL DEPENDS julius_installed grammar_kit_installed dictation_kit_installed)

install(DIRECTORY
  ${INSTALL_DIR}/bin
  ${INSTALL_DIR}/SampleGrammars
  ${INSTALL_DIR}/SampleGrammars_en
  ${INSTALL_DIR}/conf
  ${INSTALL_DIR}/include
  ${INSTALL_DIR}/lib
  ${INSTALL_DIR}/model
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
else()
install(PROGRAMS
  download_julius_data.sh
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES
  Makefile.grammar-kit
  Makefile.dictation-kit
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES
  md5sum/dictation-kit-v4.4.zip.md5sum
  md5sum/grammar-kit-v4.3.1.tar.gz.md5sum
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/md5sum)
endif()
