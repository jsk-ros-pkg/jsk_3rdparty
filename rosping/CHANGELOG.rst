^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosping
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.2 (2026-11-26)
------------------
* Merge pull request `#603 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/603>`_ from k-okada/license-spdx-ids
  Use SPDX license identifiers matching the LICENSE files (packages not built on ROS 2)
* rosping: use the SPDX identifier BSL-1.0
  <license>Boost Software License, Version 1.0</license> is BSL-1.0 in SPDX.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01Xo6jRt2Rcht9sQS6ms6A4p
* Merge pull request `#597 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/597>`_ from k-okada/ros2-nfc-rosping-zdepth
  ROS 2: nfc_ros, rosping, zdepth, zdepth_image_transport
* [rosping] add ROS2 launch test
  test/test_rosping_launch.py, the ROS2 counterpart of test-rosping.test:
  rosping pings 127.0.0.1 (instead of 8.8.8.8, which does not reply
  without the network) every 0.5 s, and a reply (delay >= 0) on
  /ping/delay is expected. As the ROS1 test, it is skipped when rosping is
  neither setuid root nor run as root (KeepAliveProc keeps the launch
  running then, as rosping exits at once).
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01Xo6jRt2Rcht9sQS6ms6A4p
* [rosping] ROS2: drop the root privileges after opening the raw socket
  Run setuid root, the ROS2 node created the shared memory files of
  Fast DDS (/dev/shm/fastrtps\_*) as root, and `ros2 topic echo /ping/delay`
  of the user received nothing (it did through UDPv4 only, with
  FASTDDS_BUILTIN_TRANSPORTS=UDPv4). The raw socket is opened by pinger,
  before the node is created, so drop the privileges (setuid(getuid()))
  then: ping/delay is received with the default transports, the socket
  still works, and the node no longer runs as root. ROS1 is unchanged.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01Xo6jRt2Rcht9sQS6ms6A4p
* [rosping] build on ROS2
  - CMakeLists.txt: the ROS2 build is prepended and the ROS1 original is
  wrapped in else()/endif() unchanged (only lines added):
  - rosping is built with rclcpp and installed to lib/rosping, for
  `ros2 run rosping rosping <host>`,
  - as on ROS1, it is made setuid root at install time when sudo works
  without a password (else the commands to do so are shown), since it
  sends ICMP ECHO_REQUEST with a raw socket,
  - the dynamic loader ignores LD_LIBRARY_PATH for setuid programs, so
  the directories of the libraries are put in its RPATH
  (CMAKE_INSTALL_RPATH_USE_LINK_PATH, and -Wl,--disable-new-dtags as on
  ROS1, as RUNPATH is not used for the dependencies of the libraries).
  - package.xml: format 3, with $ROS_VERSION conditions on the ROS1/ROS2
  dependencies and build_type; boost for ROS2 (rosboost_cfg of ROS1).
  - AMENT_IGNORE is removed.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01Xo6jRt2Rcht9sQS6ms6A4p
* [rosping] rosping.cpp: roscpp or rclcpp
  rosping.cpp runs on ROS1 and ROS2 with #if blocks, the ROS1 lines being
  unchanged: ROS1 when <ros/ros.h> is available (which defines
  ROS_VERSION_MAJOR to 1, the detection of jsk_ros1_ros2_compat's
  ros_version.h, without depending on it), else ROS2:
  - rclcpp::init(), and the arguments without the --ros-args
  (rclcpp::remove_ros_arguments()), as ros::init() removes the remappings
  from argv, so that `rosping <host>` is checked as before,
  - the "ping" node with the "rate" parameter (the publishing period in
  seconds, 10.0 by default as ROS1's ~rate) and the ping/delay
  (std_msgs/Float64) publisher, published in the same loop.
  ping.cpp (boost::asio raw ICMP socket) is unchanged.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01Xo6jRt2Rcht9sQS6ms6A4p
* Merge pull request `#584 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/584>`_ from k-okada/release-3.0.1
* Contributors: Kei Okada

3.0.1 (2026-11-10)
------------------

3.0.0 (2026-11-06)
------------------
* Merge branch 'master' into fix_hoya
* Merge branch 'master' into voicevox-ros1-ros2
* Merge pull request #575 from k-okada/ros2-support
  Add ROS2 (jazzy) support to chaplus_ros, emotion_analyzer, gdrive_ros, google_chat_ros, influxdb_store, power_switching_tools_ros, respeaker_ros, rospatlite, sesame_ros, switchbot_ros
* [GA] add ROS2 (jazzy) CI and AMENT_IGNORE packages not yet ported to ROS2
  - ros2.yaml: industrial_ci on jazzy. The ROS2 ports depend on
  jsk_ros1_ros2_compat >= 3.0.2 (jsk_common), installed by rosdep from
  its jazzy release.
  - AMENT_IGNORE the packages not ported to ROS2 yet.
  - config.yml: AMENT_IGNORE is treated as CATKIN_IGNORE by rosdep's
  package discovery, so remove them before rosdep install in the ROS1
  jobs (same as jsk_common). Also set ROS_VERSION=1 for indigo, which
  predates ros_environment.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [GA] add ROS2 (jazzy) CI and AMENT_IGNORE packages not yet ported to ROS2
  - ros2.yaml: industrial_ci on jazzy. The ROS2 ports depend on
  jsk_ros1_ros2_compat >= 3.0.2 (jsk_common), installed by rosdep from
  its jazzy release.
  - AMENT_IGNORE the packages not ported to ROS2 yet.
  - config.yml: AMENT_IGNORE is treated as CATKIN_IGNORE by rosdep's
  package discovery, so remove them before rosdep install in the ROS1
  jobs (same as jsk_common). Also set ROS_VERSION=1 for indigo, which
  predates ros_environment.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* Merge branch 'master' into power_switching_tools_ros
* Merge pull request #568 from k-okada/k-okada/add_ga_test
  Enable CI test runs and fix a few test failures
* [rosping] skip test when raw ICMP privilege is unavailable
  test-rosping.test launched the node under an extra "sudo -n" prefix on
  top of the setuid-root binary CMakeLists.txt already installs; sudo -n
  fails wherever passwordless sudo isn't configured, even though the
  setuid bit alone is sufficient (rpath is baked in, so the LD_LIBRARY_PATH
  passthrough sudo -E was carrying is unnecessary too).
  Replace the generic hztest declaration with test-rosping.py, which
  checks the installed rosping binary for setuid-root (or euid==0) and
  raises unittest.SkipTest when neither holds, instead of failing outright
  in environments where the binary hasn't been granted raw-socket
  privilege.
* Contributors: Kei Okada, Shun Hasegawa

2.1.31 (2025-05-13)
-------------------

2.1.30 (2025-05-10)
-------------------

2.1.29 (2025-01-05)
-------------------
* Adding CMAKE_EXE_LINKER_FLAGS in CMakeLists.txt to fix missing library issue while executing rosping. (`#380 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/380>`_)
* Contributors: Jules Berhault

2.1.28 (2023-07-24)
-------------------

2.1.27 (2023-06-24)
-------------------
* fix package.xml/CMakeLists.txt to supress catkin_lint errors (`#479 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/479>`_)
* Contributors: Kei Okada

2.1.26 (2023-06-14)
-------------------
* add LICENSE files (`#476 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/476>`_)
* Contributors: Kei Okada

2.1.25 (2023-06-08)
-------------------
* GithubAction: add test for  aarch64(melodic) / indigo (arm64) (`#365 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/365>`_)

  * pgm_learner/respeaker_ros/ros_speech_recognition/rosping: increase time-limit/wait-time

* Contributors: Kei Okada, Shingo Kitagawa, Yoshiki Obinata

2.1.24 (2021-07-26)
-------------------

2.1.23 (2021-07-21)
-------------------

2.1.22 (2021-06-10)
-------------------
* relax rosping test for GAf( `#245 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/245>`_)
* Contributors: Kei Okada

2.1.21 (2020-08-19)
-------------------

2.1.20 (2020-08-07)
-------------------

2.1.19 (2020-07-21)
-------------------

2.1.18 (2020-07-20)
-------------------

2.1.17 (2020-04-16)
-------------------

2.1.16 (2020-04-16)
-------------------

2.1.15 (2019-12-12)
-------------------

2.1.14 (2019-11-21)
-------------------

2.1.13 (2019-07-10)
-------------------

2.1.12 (2019-05-25)
-------------------

2.1.11 (2018-08-29)
-------------------
* run sudo -E and set LD_LIBRARY_PATH, because LD_LIBRARY_PATH is not passwed after sudo -E or suid, see set ld_library_path within (`#154 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/154>`_)
* Contributors: Kei Okada

2.1.10 (2018-04-25)
-------------------

2.1.9 (2018-04-24)
------------------

2.1.8 (2018-04-17)
------------------

2.1.7 (2018-04-09)
------------------

2.1.6 (2017-11-21)
------------------

2.1.5 (2017-11-20)
------------------

2.1.4 (2017-07-16)
------------------

2.1.3 (2017-07-07)
------------------

2.1.2 (2017-07-06)
------------------

2.1.1 (2017-07-05)
------------------

2.1.0 (2017-07-02)
------------------

2.0.20 (2017-05-09)
-------------------

2.0.19 (2017-02-22)
-------------------
* CMakeLists.txt: fix for cmake 3.5.1 (`#87 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/87>`_)
* Contributors: Kei Okada

2.0.18 (2016-10-28)
-------------------

2.0.17 (2016-10-22)
-------------------

2.0.16 (2016-10-17)
-------------------

2.0.15 (2016-10-16)
-------------------

2.0.14 (2016-03-20)
-------------------

2.0.13 (2015-12-15)
-------------------

2.0.12 (2015-11-26)
-------------------

2.0.11 (2015-10-07)
-------------------

2.0.10 (2015-10-07)
-------------------

2.0.9 (2015-09-26)
------------------

2.0.8 (2015-09-15)
------------------

2.0.7 (2015-09-14)
------------------

2.0.6 (2015-09-08)
------------------

2.0.5 (2015-08-23)
------------------

2.0.4 (2015-08-18)
------------------

2.0.3 (2015-08-01)
------------------

2.0.2 (2015-06-29)
------------------

2.0.1 (2015-06-19)
------------------

2.0.0 (2015-06-19)
------------------
* move from jsk_common to jsk_3rdparty

1.0.72 (2015-06-07)
-------------------

1.0.71 (2015-05-17)
-------------------

1.0.70 (2015-05-08)
-------------------

1.0.69 (2015-05-05)
-------------------

1.0.68 (2015-05-05)
-------------------

1.0.67 (2015-05-03)
-------------------

1.0.66 (2015-04-03)
-------------------

1.0.65 (2015-04-02)
-------------------

1.0.64 (2015-03-29)
-------------------

1.0.63 (2015-02-19)
-------------------

1.0.62 (2015-02-17)
-------------------

1.0.61 (2015-02-11)
-------------------

1.0.60 (2015-02-03)
-------------------

1.0.59 (2015-02-03)
-------------------
* Remove rosbuild files
* Contributors: Ryohei Ueda

1.0.58 (2015-01-07)
-------------------
* [rosping] Do not test on indigo, it seems to be broken
* Contributors: Ryohei Ueda

1.0.57 (2014-12-23)
-------------------

1.0.56 (2014-12-17)
-------------------

1.0.55 (2014-12-09)
-------------------

1.0.54 (2014-11-15)
-------------------

1.0.53 (2014-11-01)
-------------------

1.0.52 (2014-10-23)
-------------------

1.0.51 (2014-10-20)
-------------------

1.0.50 (2014-10-20)
-------------------

1.0.49 (2014-10-13)
-------------------

1.0.48 (2014-10-12)
-------------------

1.0.47 (2014-10-08)
-------------------

1.0.46 (2014-10-03)
-------------------

1.0.45 (2014-09-29)
-------------------

1.0.44 (2014-09-26)
-------------------

1.0.43 (2014-09-26)
-------------------

1.0.42 (2014-09-25)
-------------------

1.0.41 (2014-09-23)
-------------------

1.0.40 (2014-09-19)
-------------------

1.0.39 (2014-09-17)
-------------------

1.0.38 (2014-09-13)
-------------------

1.0.36 (2014-09-01)
-------------------

1.0.35 (2014-08-16)
-------------------

1.0.34 (2014-08-14)
-------------------

1.0.33 (2014-07-28)
-------------------

1.0.32 (2014-07-26)
-------------------

1.0.31 (2014-07-23)
-------------------

1.0.30 (2014-07-15)
-------------------

1.0.29 (2014-07-02)
-------------------

1.0.28 (2014-06-24)
-------------------

1.0.27 (2014-06-10)
-------------------

1.0.26 (2014-05-30)
-------------------

1.0.25 (2014-05-26)
-------------------

1.0.24 (2014-05-24)
-------------------

1.0.23 (2014-05-23)
-------------------

1.0.22 (2014-05-22)
-------------------

1.0.21 (2014-05-20)
-------------------

1.0.20 (2014-05-09)
-------------------
* rosping: setuid via install()
* Contributors: Kei Okada

1.0.19 (2014-05-06)
-------------------

1.0.18 (2014-05-04)
-------------------
* (rosping) Add pkg description, clarify difference with simular tool.
* Contributors: Isaac IY Saito

1.0.17 (2014-04-20)
-------------------

1.0.16 (2014-04-19)
-------------------

1.0.15 (2014-04-19)
-------------------

1.0.14 (2014-04-19)
-------------------

1.0.13 (2014-04-19)
-------------------

1.0.12 (2014-04-18)
-------------------

1.0.11 (2014-04-18)
-------------------

1.0.10 (2014-04-17)
-------------------

1.0.9 (2014-04-12)
------------------

1.0.8 (2014-04-11)
------------------

1.0.3 (2014-03-19)
------------------
* display how to set uid for rosping
* use DISDIR in install(CODE) to make rosping work
* Contributors: Kei Okada

1.0.1 (2014-03-07)
------------------
* Merge pull request `#293 <https://github.com/jsk-ros-pkg/jsk_common/issues/293>`_ from k-okada/sudo_rosping
  rosping :  add install and test
* keep persmissions during installation
* fix typo CATKIN-DEPENDS -> CATKIN_DEPENDS
* add rosdep name
* rosping: install test directory
* fix typo CATKIN-DEPENDS -> CATKIN_DEPENDS
* add rosdep name
* keep persmissions during installation
* rosping: install test directory
* Merge pull request `#283 <https://github.com/jsk-ros-pkg/jsk_common/issues/283>`_ from k-okada/release

1.0.0 (2014-03-05)
------------------
* Merge pull request `#283 <https://github.com/jsk-ros-pkg/jsk_common/issues/283>`_ from k-okada/release
  add meta package and set all package.xml to 1.0.0
* set all package to 1.0.0
* add rostest to rosping
* (rosping/catkin.cmake) try sudo see if it works
  add -n option, not to ask password
* install rosping
* fixing ping timing
* chack the arguments after ros::init
* adding ~rate parameter
* fix: display how to setuid at the end of cmake
* display how to setuid at the end of cmkae
* fix : make catkin to work rosping
* fix catkin make
* publish -1 when connection timed out
* catkinize rosping
* add rosping
* Contributors: Ryohei Ueda, Kei Okada, Yusuke Furuta
