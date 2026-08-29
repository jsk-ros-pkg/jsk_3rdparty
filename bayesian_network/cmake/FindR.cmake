# FindR.cmake
#   This module is for finding R language
#
# R_EXECUTABLE       - Path to R command
# RSCRIPT_EXECUTABLE - Path to Rscript command
# R_INCLUDE_DIR      - Path to R include directory
# R_LIBRARIES        - Path to R library
# R_LIBRARY_BASE     -
#

include(FindPackageHandleStandardArgs) # enable find_package

set(TEMP_CMAKE_FIND_APPBUNDLE ${CMAKE_FIND_APPBUNDLE})
set(CMAKE_FIND_APPBUNDLE "NEVER")

find_program(R_EXECUTABLE R DOC "R executable.")
if(R_EXECUTABLE)
  execute_process(COMMAND ${R_EXECUTABLE} RHOME
    OUTPUT_VARIABLE R_BASE_DIR
    WORKING_DIRECTORY .
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  set(R_HOME ${R_BASE_DIR} CACHE PATH "R home directory")
  mark_as_advanced(R_HOME)
endif()

find_program(RSCRIPT_EXECUTABLE Rscript DOC "Rscript executable.")

set(CMAKE_FIND_APPBUNDLE ${TEMP_CMAKE_FIND_APPBUNDLE})

find_path(R_INCLUDE_DIR R.h
  PATHS
  ${R_INCLUDE_DIR_HINT}
  /usr/local/lib
  /usr/local/lib64
  /usr/share
  /usr/share/R/include
  /usr/include
  ${R_BASE_DIR})

find_library(R_LIBRARY_BASE R
  PATHS ${R_BASE_DIR}
  PATH_SUFFIXES /lib
  DOC "R library")
set(R_LIBRARIES ${R_LIBRARY_BASE})
mark_as_advanced(RSCRIPT_EXECUTABLE R_LIBRARIES R_INCLUDE_DIR R_EXECUTABLE R_LIBRARY_BASE)

set(_REQUIRED_R_VARIABLES R_INCLUDE_DIR R_EXECUTABLE)

if(APPLE)
  list(APPEND _REQUIRED_R_VARIABLES R_LIBRARIES R_LIBRARY_BASE)
endif()

find_package_handle_standard_args(R DEFAULT_MSG ${_REQUIRED_R_VARIABLES})
