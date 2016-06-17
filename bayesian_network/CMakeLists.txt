cmake_minimum_required(VERSION 2.8.3)
project(bayesian_network)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_add_env_hooks(99.bayesian_network SHELLS bash zsh
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy
)

include(cmake/InstallBNLearn.cmake)
