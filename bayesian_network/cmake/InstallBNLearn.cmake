# InstallBNLearn.cmake

# find R
include(${PROJECT_SOURCE_DIR}/cmake/FindR.cmake)
if(NOT R_FOUND)
  message(FATAL_ERROR "R Not Found")
endif()

function(list_r_installed_packages R_INSTALLED_PACKAGES_VAR)
  set(R_LIB_PATH "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/R/site-library")
  set(ENV{R_LIBS} "${R_LIB_PATH}:$ENV{R_LIBS}")
  set(R_LIST_PACKAGES_SCRIPT "${PROJECT_SOURCE_DIR}/cmake/list_packages.r")
  execute_process(
    COMMAND ${R_LIST_PACKAGES_SCRIPT}
    OUTPUT_VARIABLE R_INSTALLED_PACKAGES
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  set(${R_INSTALLED_PACKAGES_VAR} ${R_INSTALLED_PACKAGES} PARENT_SCOPE)
endfunction()

function(find_r_package R_PKG_NAME)
  list_r_installed_packages(R_INSTALLED_PACKAGES)
  list(FIND R_INSTALLED_PACKAGES ${R_PKG_NAME} R_PKG_FIND_RESULT)
  if(${R_PKG_FIND_RESULT} EQUAL -1)
    set(R_${R_PKG_NAME}_FOUND FALSE PARENT_SCOPE)
  else()
    set(R_${R_PKG_NAME}_FOUND TRUE PARENT_SCOPE)
  endif()
endfunction()

function(install_r_package R_PKG_NAME)
  set(R_LIB_PATH "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/R/site-library")
  set(ENV{R_LIBS} "${R_LIB_PATH}:$ENV{R_LIBS}")
  message(STATUS "Appending to R_LIBS: ${R_LIB_PATH}")
  if(NOT EXISTS ${R_LIB_PATH})
    file(MAKE_DIRECTORY ${R_LIB_PATH})
  endif()
  set(R_INSTALL_PACKAGE_SCRIPT "${PROJECT_SOURCE_DIR}/cmake/install_package.r")
  execute_process(
    COMMAND ${R_INSTALL_PACKAGE_SCRIPT} ${R_PKG_NAME}
    RESULT_VARIABLE R_INSTALL_PACKAGE_RESULT
    ERROR_VARIABLE R_INSTALL_PACKAGE_ERROR
    ERROR_STRIP_TRAILING_WHITESPACE)
  if(NOT ${R_INSTALL_PACKAGE_RESULT} EQUAL 0)
    message(FATAL_ERROR "Installing R Package ${R_PKG_NAME} failed: ${R_INSTALL_PACKAGE_ERROR}")
  else()
    message(STATUS "R Package ${R_PKG_NAME} is installed")
  endif()
endfunction()

find_r_package(bnlearn)
if(NOT R_bnlearn_FOUND)
  install_r_package(bnlearn)
else()
  message("Found R Package bnlearn")
endif()
