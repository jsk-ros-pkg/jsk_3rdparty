cmake_minimum_required(VERSION 2.8.3)
project(lpg_planner)

find_package(catkin REQUIRED)

catkin_package()

if(EXISTS $ENV{HOME}/Downloads/LPG-1.2-linux.zip)
  add_custom_command(
    OUTPUT LPG-1.2-linux/lpg-1.2
    COMMAND unzip ~/Downloads/LPG-1.2-linux.zip
    )
  add_custom_command(
    OUTPUT ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/bin/lpg-1.2
    COMMAND pwd
    COMMAND ls -al
    COMMAND ls -al LPG-1.2-linux
    COMMAND cmake -E make_directory ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/bin/
    COMMAND cmake -E copy LPG-1.2-linux/lpg-1.2 ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/bin/
    DEPENDS LPG-1.2-linux/lpg-1.2
    )

  add_custom_target(lpg_planner_install ALL DEPENDS ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/bin/lpg-1.2)

  install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/bin
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    USE_SOURCE_PERMISSIONS)
else()
  message(WARNING "\n\n== Download LPG-1.2-linux.zip from http://eracle.ing.unibs.it/lpg/ ==")
endif()

