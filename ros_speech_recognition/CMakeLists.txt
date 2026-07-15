cmake_minimum_required(VERSION 3.8)
project(ros_speech_recognition)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(_default_python_executable "${Python3_EXECUTABLE}")
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

ament_package()
