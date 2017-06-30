cmake_minimum_required(VERSION 2.8.3)
project(julius_ros)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

install(
  DIRECTORY launch conf data scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

if(CATKIN_ENABLE_TESTING)
  if("$ENV{ROS_DISTRO}" STRGREATER "hydro") # >= indigo
    find_package(rostest REQUIRED)
    add_rostest(test/julius.test)
    add_rostest(test/grammar.test)
  endif()
endif()
