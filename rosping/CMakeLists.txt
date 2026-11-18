if("$ENV{ROS_VERSION}" EQUAL 2)
  cmake_minimum_required(VERSION 3.8)
  project(rosping)

  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(Boost REQUIRED COMPONENTS system)

  # rosping sends ICMP ECHO_REQUEST with a raw socket, which needs root, so
  # it is installed setuid root as on ROS1 (below). The dynamic loader
  # ignores LD_LIBRARY_PATH for setuid programs, so the directories of the
  # libraries are put in its RPATH (not RUNPATH, which is not used for
  # their dependencies)
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
  add_executable(rosping src/rosping.cpp src/ping.cpp)
  target_include_directories(rosping PRIVATE ${Boost_INCLUDE_DIRS})
  target_link_libraries(rosping ${Boost_LIBRARIES} rclcpp::rclcpp ${std_msgs_TARGETS})
  target_link_options(rosping PRIVATE "-Wl,--disable-new-dtags")

  install(TARGETS rosping
    DESTINATION lib/${PROJECT_NAME}
  )
  # setuid root, when sudo works without a password, as on ROS1
  install(CODE "execute_process(COMMAND sudo -n sh -c \"cd \$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME} && chown root:root rosping && chmod 4755 rosping\" RESULT_VARIABLE _ret)
    if(NOT _ret EQUAL 0)
      message(STATUS \"rosping is not setuid root, run: sudo chown root:root \$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/rosping; sudo chmod 4755 \$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/rosping\")
    endif()
  ")

  if(BUILD_TESTING)
    find_package(launch_testing_ament_cmake REQUIRED)
    add_launch_test(test/test_rosping_launch.py)
  endif()

  ament_package()

else() # if("$ENV{ROS_VERSION}" EQUAL 2)
# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(rosping)

SET(CMAKE_EXE_LINKER_FLAGS "-Wl,--disable-new-dtags")

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs rostest)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS roscpp std_msgs
    INCLUDE_DIRS
    LIBRARIES
)

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_executable(rosping src/rosping.cpp src/ping.cpp)
target_link_libraries(rosping ${Boost_LIBRARIES} ${catkin_LIBRARIES})

string(ASCII 27 ESCAPE)
add_custom_command(
  OUTPUT message
  COMMAND sudo -n sh -c 'cd ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}\; chown root.root rosping\; chmod 4755 rosping' || (echo "${ESCAPE}[34m#\ type\ following\ command\ before\ execute\ rosping...\ sudo\ chown\ root.root\ ./bin/rosping\;\ sudo\ chmod 4755\ ./bin/rosping${ESCAPE}[0m")
  DEPENDS bin/rosping)
add_custom_target(message_all ALL DEPENDS message)

install(
  PROGRAMS ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/rosping
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE SETUID
)
install(CODE
  "execute_process(COMMAND sudo -n sh -c \"cd \$ENV{DESTDIR}/${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION} && chown root.root rosping && ls -al rosping && chmod 4755 rosping && ls -al rosping\")
")

install(DIRECTORY test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )
if (CATKIN_ENABLE_TESTING AND (NOT $ENV{ROS_DISTRO} STREQUAL "indigo"))
  add_rostest(test/test-rosping.test)
endif(CATKIN_ENABLE_TESTING AND (NOT $ENV{ROS_DISTRO} STREQUAL "indigo"))

endif() # if("$ENV{ROS_VERSION}" EQUAL 2)
