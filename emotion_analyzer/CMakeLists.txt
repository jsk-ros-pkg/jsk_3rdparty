if("$ENV{ROS_VERSION}" EQUAL 2)
  cmake_minimum_required(VERSION 3.8)
  project(emotion_analyzer)

  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_python REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(std_msgs REQUIRED)

  rosidl_generate_interfaces(${PROJECT_NAME}
    srv/AnalyzeText.srv
    srv/AnalyzeAudio.srv
    DEPENDENCIES std_msgs
  )

  # rosidl_generate_interfaces() already installs the emotion_analyzer
  # python package (emotion_analyzer.srv), so add src/emotion_analyzer/utils
  # into it rather than calling ament_python_install_package() again.
  install(DIRECTORY src/${PROJECT_NAME}/utils
    DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
    PATTERN "*.pyc" EXCLUDE
    PATTERN "__pycache__" EXCLUDE
  )

  # hume is not available from apt, and catkin_virtualenv has no ROS2 port,
  # so the services run in a uv venv made from pyproject.toml/uv.lock.
  find_package(jsk_ros1_ros2_compat REQUIRED)
  jsk_ros1_ros2_compat_generate_uv_venv(CHECK_IMPORTS hume)
  jsk_ros1_ros2_compat_install_python(
    PROGRAMS scripts/analyze_audio_service.py scripts/analyze_text_service.py
    USE_VENV)

  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )

  ament_package()

else() # if("$ENV{ROS_VERSION}" EQUAL 2)
cmake_minimum_required(VERSION 2.8.3)
project(emotion_analyzer)

find_package(catkin REQUIRED COMPONENTS
  catkin_virtualenv
  std_msgs
  message_generation
  )

catkin_python_setup()

add_service_files(
  FILES
  AnalyzeText.srv
  AnalyzeAudio.srv
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  )

catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime
)

find_package(Python3 3.8 QUIET COMPONENTS Interpreter)
if(NOT Python3_FOUND)
  message(WARNING "emotion_analyzer (Hume AI) requires python3.8 or newer")
  return()
endif()
message(STATUS "Found Python: ${Python3_EXECUTABLE}")
message(STATUS "Python Version: ${Python3_VERSION}")

catkin_generate_virtualenv(
  INPUT_REQUIREMENTS requirements.in
  PYTHON_INTERPRETER python3
  CHECK_VENV FALSE
)

file(GLOB PYTHON_SCRIPTS scripts/*.py)
catkin_install_python(PROGRAMS ${PYTHON_SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

endif() # if("$ENV{ROS_VERSION}" EQUAL 2)
