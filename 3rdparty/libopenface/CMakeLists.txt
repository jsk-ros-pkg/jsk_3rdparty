cmake_minimum_required(VERSION 2.8.3)
project(libopenface)

find_package(catkin REQUIRED mk)

add_custom_target(libopenface ALL
  DEPENDS ${CATKIN_DEVEL_PREFIX}/lib/libFaceAnalyzer.a
          ${CATKIN_DEVEL_PREFIX}/lib/libLandmarkDetector.a)
add_custom_command(OUTPUT ${CATKIN_DEVEL_PREFIX}/lib/libFaceAnalyzer.a ${CATKIN_DEVEL_PREFIX}/lib/libLandmarkDetector.a
  DEPENDS Makefile
  COMMAND cmake -E chdir ${CMAKE_CURRENT_BINARY_DIR} make -f ${PROJECT_SOURCE_DIR}/Makefile MK_DIR=${mk_PREFIX}/share/mk INSTALL_DIR=${CATKIN_DEVEL_PREFIX} MD5SUM_DIR=${PROJECT_SOURCE_DIR})

# fake install directory for catkin_package
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/openface)
if("$ENV{ROS_DISTRO}" STREQUAL "groovy")
  catkin_package(
    INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include/openface
    LIBRARIES    openface
    SKIP_CMAKE_CONFIG_GENERATION
    )
  set(${PROJECT_NAME}_EXPORTED_TARGETS libopenface)
else()
  catkin_package(
    INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include/openface
    LIBRARIES    openface
    EXPORTED_TARGETS libopenface
    SKIP_CMAKE_CONFIG_GENERATION
    )
endif()



install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/openface
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
