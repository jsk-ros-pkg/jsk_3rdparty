cmake_minimum_required(VERSION 2.8.3)
project(google_cloud_texttospeech)

find_package(catkin REQUIRED)

catkin_add_env_hooks(50.google_cloud_texttospeech SHELLS sh
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

catkin_package()

# python3 only; python2 environments install google-cloud-texttospeech with pip
if("$ENV{ROS_PYTHON_VERSION}" STREQUAL "3")
  find_package(catkin_virtualenv REQUIRED)
  catkin_generate_virtualenv(
    PYTHON_INTERPRETER python3
    CHECK_VENV FALSE
  )
  install(FILES requirements.txt
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endif()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

catkin_install_python(PROGRAMS bin/text2wave 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/bin)
