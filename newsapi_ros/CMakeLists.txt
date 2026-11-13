cmake_minimum_required(VERSION 2.8.3)
project(newsapi_ros)

find_package(
  catkin REQUIRED COMPONENTS
  catkin_virtualenv REQUIRED
  rospy
  actionlib_msgs
  std_msgs
  message_generation
  )

add_message_files(
  FILES
  News.msg
)

add_action_files(
  FILES
  GetTopHeadlines.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package()

catkin_generate_virtualenv(
  PYTHON_INTERPRETER python3
  )

file(GLOB PYTHON_NODES node_scripts/*.py)
catkin_install_python(
  PROGRAMS ${PYTHON_NODES}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES requirements.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
