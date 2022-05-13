cmake_minimum_required(VERSION 2.8.3)
project(dialogflow_ros)

if($ENV{ROS_DISTRO} STREQUAL "indigo")
  message(WARNING "following requirements.txt syntax is not support on 14.04")
  message(WARNING "google-api-core[grpc]==1.31.5  # via google-cloud-language")
  find_package(catkin)
  catkin_package()
  return()
endif()
execute_process(COMMAND bash -c "gcc -dumpmachine" OUTPUT_VARIABLE gcc_dump_machine OUTPUT_STRIP_TRAILING_WHITESPACE)
message("-- gcc dumpmachine returns ${gcc_dump_machine}")
if(NOT gcc_dump_machine MATCHES "x86_64-.*")
  message(WARNING "pip -i requirements.txt work only with i686 ???")
  message(WARNING "`pip install grpcio` fails with
  third_party/boringssl-with-bazel/src/crypto/hrss/asm/poly_rq_mul.S: Assembler messages:
  third_party/boringssl-with-bazel/src/crypto/hrss/asm/poly_rq_mul.S:306: Error: bad register name `%rbp'
  third_party/boringssl-with-bazel/src/crypto/hrss/asm/poly_rq_mul.S:308: Error: bad register expression
  third_party/boringssl-with-bazel/src/crypto/hrss/asm/poly_rq_mul.S:309: Error: bad register name `%rsp'")
  find_package(catkin)
  catkin_package()
  return()
endif()

find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
    actionlib_msgs
    catkin_virtualenv
)

add_message_files(
  FILES
  DialogResponse.msg
)

add_action_files(
  FILES
  DialogText.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

catkin_generate_virtualenv(INPUT_REQUIREMENTS requirements.in)

file(GLOB NODE_SCRIPTS_FILES node_scripts/*.py)
catkin_install_python(
  PROGRAMS ${NODE_SCRIPTS_FILES}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
