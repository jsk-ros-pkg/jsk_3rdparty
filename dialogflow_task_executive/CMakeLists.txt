cmake_minimum_required(VERSION 2.8.3)
project(dialogflow_task_executive)

execute_process(COMMAND bash -c "gcc -dumpmachine" OUTPUT_VARIABLE gcc_dump_machine OUTPUT_STRIP_TRAILING_WHITESPACE)
message("-- gcc dumpmachine returns ${gcc_dump_machine}")
if(NOT (gcc_dump_machine MATCHES "x86_64-.*" OR gcc_dump_machine MATCHES "aarch64-.*"))
  message(WARNING "pip -i requirements.txt work only with i686 ???")
  message(WARNING "`pip install grpcio` fails with
  third_party/boringssl-with-bazel/src/crypto/hrss/asm/poly_rq_mul.S: Assembler messages:
  third_party/boringssl-with-bazel/src/crypto/hrss/asm/poly_rq_mul.S:306: Error: bad register name `%rbp'
  third_party/boringssl-with-bazel/src/crypto/hrss/asm/poly_rq_mul.S:308: Error: bad register expression
  third_party/boringssl-with-bazel/src/crypto/hrss/asm/poly_rq_mul.S:309: Error: bad register name `%rsp'")
endif()

find_package(catkin REQUIRED COMPONENTS
    message_generation
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
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

if(NOT(gcc_dump_machine MATCHES "x86_64-.*" OR gcc_dump_machine MATCHES "aarch64-.*"))
  message(WARNING "pip -i requirements.txt with grpcio works only with i686, so create dummy requirements.txt and run 'catkin_generate_virtualenv' for ${gcc_dump_machine}")
  file(WRITE ${CMAKE_CURRENT_SOURCE_DIR}/requirements.txt "")
  catkin_generate_virtualenv(CHECK_VENV FALSE)
elseif("$ENV{ROS_DISTRO}" STREQUAL "indigo")
  message(WARNING "following requirements.txt syntax is not support on 14.04")
  message(WARNING "google-api-core[grpc]==1.31.5  # via google-cloud-language")
  message(WARNING "so we intentionally use requirements.txt")
  message(WARNING "To use this program, we need to run")
  message(WARNING "sed -i 's/upgrade_pip=True/upgrade_pip=False/' /opt/ros/indigo/share/catkin_virtualenv/cmake/build_venv.py")
  message(WARNING "to prevent upgrading to latest pip, which is not Python2 compatible")
  file(COPY requirements.txt.indigo DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
  # github build failed with invalid device link
  # file(RENAME ${CMAKE_CURRENT_BINARY_DIR}/requirements.txt.indigo requirements.txt)
  execute_process(
    COMMAND cp ${CMAKE_CURRENT_BINARY_DIR}/requirements.txt.indigo ${CMAKE_CURRENT_SOURCE_DIR}/requirements.txt
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
  file(DOWNLOAD http://curl.haxx.se/ca/cacert.pem ${CMAKE_BINARY_DIR}/cacert.pem)
  catkin_generate_virtualenv(PYTHON_INTERPRETER python2
    # https://stackoverflow.com/questions/25981703/pip-install-fails-with-connection-error-ssl-certificate-verify-failed
    EXTRA_PIP_ARGS -vvv --cert=${CMAKE_BINARY_DIR}/cacert.pem)
elseif("$ENV{ROS_DISTRO}" STREQUAL "kinetic" OR "$ENV{ROS_DISTRO}" STREQUAL "melodic")
  catkin_generate_virtualenv(
    INPUT_REQUIREMENTS requirements.in
    PYTHON_INTERPRETER python2
    )
elseif("$ENV{ROS_DISTRO}" STREQUAL "noetic")
  catkin_generate_virtualenv(
    INPUT_REQUIREMENTS requirements.in.noetic
    PYTHON_INTERPRETER python3
    )
else()
  catkin_generate_virtualenv(
    INPUT_REQUIREMENTS requirements.in.python3.12
    PYTHON_INTERPRETER python3
    )
endif()

file(GLOB PYTHON_SCRIPT_FILES node_scripts/*.py test/*.py)
catkin_install_python(
  PROGRAMS ${PYTHON_SCRIPT_FILES}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch apps samples config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
  if(NOT (gcc_dump_machine MATCHES "x86_64-.*" OR gcc_dump_machine MATCHES "aarch64-.*"))
    message(WARNING "pip -i requirements.txt work only with i686, so skipping test for ${gcc_dump_machine}")
  else()
    add_rostest(test/test_rospy_node.test
      DEPENDENCIES ${PROJECT_NAME}_generate_virtualenv ${PROJECT_NAME}_generate_messages
    )
  endif()
endif()
