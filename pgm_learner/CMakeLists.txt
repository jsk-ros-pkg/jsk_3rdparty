cmake_minimum_required(VERSION 2.8.3)
project(pgm_learner)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
)

catkin_python_setup()

add_message_files(
  FILES
  ConditionalProbability.msg
  GraphStructure.msg
  GraphEdge.msg
  DiscreteNode.msg
  DiscreteGraphState.msg
  DiscreteNodeState.msg
  LinearGaussianNode.msg
  LinearGaussianGraphState.msg
  LinearGaussianNodeState.msg
)

add_service_files(
  FILES
  DiscreteQuery.srv
  DiscreteParameterEstimation.srv
  DiscreteStructureEstimation.srv
  LinearGaussianParameterEstimation.srv
  LinearGaussianStructureEstimation.srv
)

generate_messages(
  DEPENDENCIES
)

catkin_package()

install(
  DIRECTORY launch sample scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

if(CATKIN_ENABLE_TESTING)
  # python-libpgm-pip did not work with python3
  if ("$ENV{ROS_PYTHON_VERSION}" STREQUAL "2")
    find_package(rostest REQUIRED)
    add_rostest(test/test_discrete_bn.test)
    add_rostest(test/test_lg_bn.test)
  endif()
endif()
