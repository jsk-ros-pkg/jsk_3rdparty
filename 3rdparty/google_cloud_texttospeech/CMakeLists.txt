cmake_minimum_required(VERSION 2.8.3)
project(google_cloud_texttospeech)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

catkin_install_python(PROGRAMS bin/text2wave 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/bin)
