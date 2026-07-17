cmake_minimum_required(VERSION 3.8)
project(ros_speech_recognition)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

function(ros_speech_recognition_find_uv out_var)
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

function(ros_speech_recognition_run_uv)
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

ros_speech_recognition_find_uv(UV_EXECUTABLE)
set(_venv_dir "${CMAKE_CURRENT_SOURCE_DIR}/.venv")
ros_speech_recognition_run_uv(
  venv --system-site-packages --python "${Python3_EXECUTABLE}"
  --allow-existing "${_venv_dir}"
)
ros_speech_recognition_run_uv(
  sync --python "${_venv_dir}/bin/python3"
)

set(_default_python_executable "${_venv_dir}/bin/python3")
if(DEFINED ENV{ROS_SPEECH_RECOGNITION_PYTHON_EXECUTABLE})
  set(_default_python_executable "$ENV{ROS_SPEECH_RECOGNITION_PYTHON_EXECUTABLE}")
endif()

set(
  ROS_SPEECH_RECOGNITION_PYTHON_EXECUTABLE
  "${_default_python_executable}"
  CACHE FILEPATH
  "Python interpreter written into installed ROS executable shebangs"
)

if(NOT IS_ABSOLUTE "${ROS_SPEECH_RECOGNITION_PYTHON_EXECUTABLE}")
  message(FATAL_ERROR
    "ROS_SPEECH_RECOGNITION_PYTHON_EXECUTABLE must be an absolute path: "
    "${ROS_SPEECH_RECOGNITION_PYTHON_EXECUTABLE}")
endif()

if(NOT EXISTS "${ROS_SPEECH_RECOGNITION_PYTHON_EXECUTABLE}")
  message(FATAL_ERROR
    "ROS_SPEECH_RECOGNITION_PYTHON_EXECUTABLE does not exist: "
    "${ROS_SPEECH_RECOGNITION_PYTHON_EXECUTABLE}")
endif()

message(STATUS
  "ros_speech_recognition executable shebang: "
  "${ROS_SPEECH_RECOGNITION_PYTHON_EXECUTABLE}")

ament_python_install_package(
  ${PROJECT_NAME}
  PACKAGE_DIR src/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.launch.xml"
)

set(_generated_script_dir "${CMAKE_CURRENT_BINARY_DIR}/generated_scripts")
file(MAKE_DIRECTORY "${_generated_script_dir}")

function(ros_speech_recognition_install_script executable_name module_name)
  set(_script_path "${_generated_script_dir}/${executable_name}")
  file(WRITE "${_script_path}" "\
#!${ROS_SPEECH_RECOGNITION_PYTHON_EXECUTABLE}
from ${module_name} import main


if __name__ == '__main__':
    main()
")
  file(CHMOD "${_script_path}"
    PERMISSIONS
      OWNER_READ OWNER_WRITE OWNER_EXECUTE
      GROUP_READ GROUP_EXECUTE
      WORLD_READ WORLD_EXECUTE
  )
  install(PROGRAMS "${_script_path}" DESTINATION lib/${PROJECT_NAME})
endfunction()

ros_speech_recognition_install_script(
  install_trained_data
  ros_speech_recognition.install_trained_data
)
ros_speech_recognition_install_script(
  parrot_node
  ros_speech_recognition.parrot_node
)
ros_speech_recognition_install_script(
  speech_recognition_candidates_to_string
  ros_speech_recognition.candidates_to_string
)
ros_speech_recognition_install_script(
  speech_recognition_node
  ros_speech_recognition.node
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_node test/test_node.py
    PYTHON_EXECUTABLE "${_venv_dir}/bin/python3"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
endif()

ament_package()
