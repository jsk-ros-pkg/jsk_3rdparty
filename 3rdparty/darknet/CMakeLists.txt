cmake_minimum_required(VERSION 2.8.3)
project(darknet)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CFG_EXTRAS darknet-extras.cmake)

include(ExternalProject)
externalproject_add(${PROJECT_NAME}_ext
    GIT_REPOSITORY https://github.com/pjreddie/darknet.git
    GIT_TAG 2710d632571a0083f6b8aa57a3c822b57a9f4866 # same as master (2016.12.01)
    TIMEOUT 120
    BUILD_IN_SOURCE true
    PATCH_COMMAND bash -c "cp -f ${PROJECT_SOURCE_DIR}/cmake/darknet.cmake CMakeLists.txt"
    CMAKE_ARGS "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE};-DCMAKE_INSTALL_PREFIX=${PROJECT_SOURCE_DIR}"
    LOG_DOWNLOAD true
    LOG_BUILD true
    LOG_INSTALL true)

set(DARKNET_LIB_PATH "${PROJECT_SOURCE_DIR}/lib/libdarknet.so")
set(DARKNET_BIN_PATH "${PROJECT_SOURCE_DIR}/bin/darknet")
add_library(${PROJECT_NAME} SHARED IMPORTED)
set_target_properties(${PROJECT_NAME} PROPERTIES
  IMPORTED_LOCATION ${DARKNET_LIB_PATH})
add_dependencies(${PROJECT_NAME} darknet_ext)

# see http://answers.ros.org/question/223866/cmakecatkin-how-to-export-imported-target/
add_custom_command(TARGET ${PROJECT_NAME}_ext
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy ${DARKNET_LIB_PATH} ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/lib${PROJECT_NAME}${CMAKE_SHARED_LIBRARY_SUFFIX})

function(download_weights network_name)
  if("${network_name}" STREQUAL "yolo")
    set(url "http://www.jsk.t.u-tokyo.ac.jp/~wada/data/yolo.weights")
  else()
    set(url "http://pjreddie.com/media/files/${network_name}.weights")
  endif()
  file(DOWNLOAD ${url}
    ${PROJECT_SOURCE_DIR}/weights/${network_name}.weights
    TIMEOUT 120)
endfunction()

# download pre-trained weights
download_weights(yolo)
# download_weights(tiny-yolo-voc)
# download_weights(yolo-voc)


install(FILES ${DARKNET_LIB_PATH} DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(FILES ${DARKNET_BIN_PATH} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY weights data cfg cmake test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test)
endif()
