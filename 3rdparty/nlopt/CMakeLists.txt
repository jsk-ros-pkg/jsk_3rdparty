project(nlopt)

cmake_minimum_required(VERSION 2.4.6)

find_package(catkin REQUIRED COMPONENTS mk)

add_custom_target(libnlopt_cxx ALL
  DEPENDS ${CATKIN_DEVEL_PREFIX}/lib/libnlopt_cxx.so)
add_custom_command(OUTPUT
  ${CATKIN_DEVEL_PREFIX}/lib/libnlopt_cxx.so
  COMMAND mkdir -p ${CMAKE_CURRENT_BINARY_DIR}/build
  COMMAND cp ${PROJECT_SOURCE_DIR}/build/nlopt-2.3.tar.gz ${CMAKE_CURRENT_BINARY_DIR}/build
  COMMAND cp ${PROJECT_SOURCE_DIR}/nlopt-2.3.tar.gz.md5sum ${CMAKE_CURRENT_BINARY_DIR}
  COMMAND mkdir -p ${CMAKE_CURRENT_BINARY_DIR}/tmp
  COMMAND cmake -E chdir ${CMAKE_CURRENT_BINARY_DIR} make -f ${PROJECT_SOURCE_DIR}/Makefile DSTDIR=${CMAKE_CURRENT_BINARY_DIR}/tmp MK_DIR=${mk_PREFIX}/share/mk
  # copy nlopt.h and nlopt.hpp
  COMMAND mkdir -p  ${CATKIN_DEVEL_PREFIX}/include/nlopt
  COMMAND cp -r ${CMAKE_CURRENT_BINARY_DIR}/tmp/include/* ${CATKIN_DEVEL_PREFIX}/include/nlopt
  # copy libnlopt
  COMMAND cp -a ${CMAKE_CURRENT_BINARY_DIR}/tmp/lib/*.so* ${CATKIN_DEVEL_PREFIX}/lib/
  COMMAND cp -r ${CMAKE_CURRENT_BINARY_DIR}/tmp/share ${CATKIN_DEVEL_PREFIX}/share/
  DEPENDS Makefile)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS
    INCLUDE_DIRS
    LIBRARIES nlopt_cxx
    EXPORTED_TARGETS libnlopt_cxx
    CFG_EXTRAS nlopt-extras.cmake
)

# set(NLOPTDIR ${CATKIN_PACKAGE_SHARE_DESTINATION}/jskeus/eus)
# install(CODE "message(\"-- Force change Cflags of \$ENV{DESTDIR}/\${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/pkgconfig/euslisp.pc \$ENV{DESTDIR}/\${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake/\${PROJECT_NAME}Config.cmake\") execute_process(COMMAND sed -i s@\${CMAKE_INSTALL_PREFIX}/include@\${CMAKE_INSTALL_PREFIX}/${EUSDIR}/include@ \$ENV{DESTDIR}/\${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/pkgconfig/euslisp.pc \$ENV{DESTDIR}/\${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake/${PROJECT_NAME}Config.cmake)")

# install(DIRECTORY lib
#   DESTINATION ${EUSDIR}/lib
#   FILES_MATCHING PATTERN "*.l" PATTERN ".svn" EXCLUDE
# )

# install(DIRECTORY test
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#   USE_SOURCE_PERMISSIONS
#   )

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/nlopt/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib/
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  FILES_MATCHING PATTERN "*nlopt*")

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/share/nlopt/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
