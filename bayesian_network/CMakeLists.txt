cmake_minimum_required(VERSION 2.8.3)
project(bayesian_network)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
  CATKIN_DEPENDS rospy
)

include(cmake/InstallBNLearn.cmake)

