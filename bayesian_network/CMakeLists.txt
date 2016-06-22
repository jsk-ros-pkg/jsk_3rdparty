cmake_minimum_required(VERSION 2.8.3)
project(bayesian_network)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
)

catkin_add_env_hooks(99.bayesian_network SHELLS bash zsh
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

catkin_python_setup()

add_message_files(FILES
  DiscreteNode.msg
  LinearGaussianNode.msg
  Evidence.msg
)

add_service_files(FILES
  Predict.srv
  Query.srv
  Sample.srv
  StructureEstimation.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime rospy std_msgs
)

include(cmake/InstallBNLearn.cmake)

install(DIRECTORY script sample launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
