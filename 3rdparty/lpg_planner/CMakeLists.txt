cmake_minimum_required(VERSION 2.8.3)
project(lpg_planner)

find_package(catkin REQUIRED)
catkin_package()

if(EXISTS $ENV{HOME}/Downloads/lpg1.2-linux.tar.gz)
  add_custom_command(
    OUTPUT LPG-1.2-linux/lpg-1.2
    COMMAND tar xvf ~/Downloads/lpg1.2-linux.tar.gz
    )
  add_custom_command(
    OUTPUT ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/bin/lpg-1.2
    COMMAND cmake -E make_directory ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/bin/
    COMMAND cmake -E copy LPG-1.2-linux/lpg-1.2 ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/bin/
    DEPENDS LPG-1.2-linux/lpg-1.2
    )
  add_custom_target(build_all ALL DEPENDS ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/bin/lpg-1.2)
else()
  message(WARNING "\n\n== Download lpg1.2-linux.tar.gz from http://eracle.ing.unibs.it/lpg/download-lpg1.2.html\nDirect link (http://eracle.ing.unibs.it/lpg/lpg1.2-linux.tar.gz) ==")
endif()

