cmake_minimum_required(VERSION 3.8)
project(voicevox)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(roseus QUIET)
find_package(sound_play_msgs REQUIRED)

function(voicevox_find_uv out_var)
  find_program(_uv_executable uv)
  if(NOT _uv_executable)
    set(_uv_install_dir "${CMAKE_CURRENT_BINARY_DIR}/uv")
    find_program(_curl_executable curl)
    find_program(_wget_executable wget)
    if(_curl_executable)
      set(_uv_install_command
        "${_curl_executable} -LsSf https://astral.sh/uv/install.sh | env UV_INSTALL_DIR='${_uv_install_dir}' INSTALLER_NO_MODIFY_PATH=1 sh")
    elseif(_wget_executable)
      set(_uv_install_command
        "${_wget_executable} -qO- https://astral.sh/uv/install.sh | env UV_INSTALL_DIR='${_uv_install_dir}' INSTALLER_NO_MODIFY_PATH=1 sh")
    else()
      message(FATAL_ERROR "uv was not found and neither curl nor wget is available")
    endif()
    message(STATUS "uv was not found; installing uv into ${_uv_install_dir}")
    execute_process(
      COMMAND bash -c "${_uv_install_command}"
      RESULT_VARIABLE _uv_install_result
      OUTPUT_VARIABLE _uv_install_output
      ERROR_VARIABLE _uv_install_error
    )
    if(NOT _uv_install_result EQUAL 0)
      message(FATAL_ERROR
        "Failed to install uv:\n${_uv_install_output}\n${_uv_install_error}")
    endif()
    if(EXISTS "${_uv_install_dir}/uv")
      set(_uv_executable "${_uv_install_dir}/uv")
    elseif(EXISTS "${_uv_install_dir}/bin/uv")
      set(_uv_executable "${_uv_install_dir}/bin/uv")
    else()
      message(FATAL_ERROR "uv installer completed but uv executable was not found")
    endif()
  endif()
  set(${out_var} "${_uv_executable}" PARENT_SCOPE)
endfunction()

function(voicevox_run_uv)
  execute_process(
    COMMAND "${UV_EXECUTABLE}" ${ARGN}
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    RESULT_VARIABLE _uv_result
    OUTPUT_VARIABLE _uv_output
    ERROR_VARIABLE _uv_error
  )
  if(NOT _uv_result EQUAL 0)
    message(FATAL_ERROR "uv ${ARGN} failed:\n${_uv_output}\n${_uv_error}")
  endif()
endfunction()

execute_process(
  COMMAND bash -c "gcc -dumpmachine"
  OUTPUT_VARIABLE gcc_dump_machine
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
message(STATUS "gcc dumpmachine returns ${gcc_dump_machine}")
if(gcc_dump_machine MATCHES "x86_64-.*")
  set(ARCH x64)
elseif(gcc_dump_machine MATCHES "aarch64-.*")
  set(ARCH arm64)
else()
  message(WARNING "unsupported platform")
  return()
endif()

message(STATUS "Found Python: ${Python3_EXECUTABLE}")
message(STATUS "Python Version: ${Python3_VERSION}")

if(Python3_VERSION VERSION_LESS "3.12")
  message(FATAL_ERROR "voicevox ROS 2 build requires Python 3.12 or newer")
endif()

set(CORE_VERSION 0.15.7)
set(CORE_HASH_x64   "444dc362d98e065b8581e5a9e403b8fc")
set(CORE_HASH_arm64 "fd33e2bf200119f70dece4b6ab58b3b0")
set(ENGINE_VERSION 0.23.0)

voicevox_find_uv(UV_EXECUTABLE)
set(_venv_dir "${CMAKE_CURRENT_SOURCE_DIR}/.venv")
voicevox_run_uv(
  venv --system-site-packages --python "${Python3_EXECUTABLE}"
  --allow-existing "${_venv_dir}"
)
set(_venv_python "${_venv_dir}/bin/python3")
voicevox_run_uv(
  sync --python "${_venv_python}"
)

set(_default_python_executable "${_venv_python}")
if(DEFINED ENV{VOICEVOX_PYTHON_EXECUTABLE})
  set(_default_python_executable "$ENV{VOICEVOX_PYTHON_EXECUTABLE}")
endif()

set(
  VOICEVOX_PYTHON_EXECUTABLE
  "${_default_python_executable}"
  CACHE FILEPATH
  "Python interpreter written into installed voicevox executable shebangs"
)

if(NOT IS_ABSOLUTE "${VOICEVOX_PYTHON_EXECUTABLE}")
  message(FATAL_ERROR
    "VOICEVOX_PYTHON_EXECUTABLE must be an absolute path: "
    "${VOICEVOX_PYTHON_EXECUTABLE}")
endif()

if(NOT EXISTS "${VOICEVOX_PYTHON_EXECUTABLE}")
  message(FATAL_ERROR
    "VOICEVOX_PYTHON_EXECUTABLE does not exist: "
    "${VOICEVOX_PYTHON_EXECUTABLE}")
endif()

include(ExternalProject)

set(VOICEVOX_TARGET_DIR ${CMAKE_CURRENT_BINARY_DIR}/voicevox)
ExternalProject_Add(voicevox_engine
  GIT_REPOSITORY    https://github.com/VOICEVOX/voicevox_engine
  GIT_TAG           ${ENGINE_VERSION}
  BUILD_IN_SOURCE   TRUE
  CONFIGURE_COMMAND ""
  BUILD_COMMAND
    COMMAND test -e speaker_info && mv speaker_info speaker_info.orig || echo "OK"
    COMMAND test -e resources/character_info && mv resources/character_info resources/character_info.orig || echo "OK"
  INSTALL_COMMAND   ${CMAKE_COMMAND} -E copy_directory ./ ${VOICEVOX_TARGET_DIR}/voicevox_engine
)
ExternalProject_Add(voicevox_resource
  GIT_REPOSITORY    https://github.com/VOICEVOX/voicevox_resource
  GIT_TAG           main
  GIT_PROGRESS      TRUE
  GIT_SHALLOW       TRUE
  BUILD_IN_SOURCE   TRUE
  CONFIGURE_COMMAND ""
  BUILD_COMMAND     ""
  INSTALL_COMMAND
    COMMAND test -e ${VOICEVOX_TARGET_DIR}/voicevox_engine/speaker_info || ${Python3_EXECUTABLE} ./scripts/clean_character_info.py --output_dir ${VOICEVOX_TARGET_DIR}/voicevox_engine/speaker_info
    COMMAND test -e ${VOICEVOX_TARGET_DIR}/voicevox_engine/resources/character_info || ${Python3_EXECUTABLE} ./scripts/clean_character_info.py --output_dir ${VOICEVOX_TARGET_DIR}/voicevox_engine/resources/character_info
  DEPENDS voicevox_engine
)
ExternalProject_Add(voicevox_core
  URL               https://github.com/VOICEVOX/voicevox_core/releases/download/${CORE_VERSION}/voicevox_core-linux-${ARCH}-cpu-${CORE_VERSION}.zip
  URL_HASH          MD5=${CORE_HASH_${ARCH}}
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  BUILD_IN_SOURCE   TRUE
  CONFIGURE_COMMAND ""
  BUILD_COMMAND     ""
  INSTALL_COMMAND   ${CMAKE_COMMAND} -E copy_directory ./ ${VOICEVOX_TARGET_DIR}/voicevox_core
)

set(ENGINE_PATH ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME})
set(VENV_PATH ${_venv_dir})
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/bin/run-voicevox.in
  ${CMAKE_CURRENT_BINARY_DIR}/bin/run-voicevox
  @ONLY
)

file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/bin/text2wave" "\
#!/bin/bash
exec ros2 run voicevox request_synthesis.py \"$@\"
")
file(CHMOD "${CMAKE_CURRENT_BINARY_DIR}/bin/text2wave"
  PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE
    GROUP_READ GROUP_EXECUTE
    WORLD_READ WORLD_EXECUTE
)

file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/node_scripts")
foreach(_script request_synthesis.py list_speakers.py)
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/node_scripts/${_script}" "\
#!${VOICEVOX_PYTHON_EXECUTABLE}
import runpy

runpy.run_path('${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/node_scripts/${_script}', run_name='__main__')
")
  file(CHMOD "${CMAKE_CURRENT_BINARY_DIR}/node_scripts/${_script}"
    PERMISSIONS
      OWNER_READ OWNER_WRITE OWNER_EXECUTE
      GROUP_READ GROUP_EXECUTE
      WORLD_READ WORLD_EXECUTE
  )
endforeach()

install(
  PROGRAMS
    ${CMAKE_CURRENT_BINARY_DIR}/node_scripts/request_synthesis.py
    ${CMAKE_CURRENT_BINARY_DIR}/node_scripts/list_speakers.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    ${CMAKE_CURRENT_BINARY_DIR}/bin/run-voicevox
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    ${CMAKE_CURRENT_BINARY_DIR}/bin/text2wave
    ${CMAKE_CURRENT_BINARY_DIR}/bin/run-voicevox
  DESTINATION share/${PROJECT_NAME}/bin
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.launch.xml"
)

install(
  DIRECTORY roseus
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

install(
  DIRECTORY node_scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

install(
  DIRECTORY
    ${VOICEVOX_TARGET_DIR}/voicevox_engine
    ${VOICEVOX_TARGET_DIR}/voicevox_core
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_request_synthesis test/test_request_synthesis.py
    PYTHON_EXECUTABLE "${_venv_python}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
endif()

ament_package()
