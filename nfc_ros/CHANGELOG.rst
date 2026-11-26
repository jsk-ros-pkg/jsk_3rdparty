^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package nfc_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.2 (2026-11-26)
------------------
* Merge pull request `#597 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/597>`_ from k-okada/ros2-nfc-rosping-zdepth
  ROS 2: nfc_ros, rosping, zdepth, zdepth_image_transport
* [nfc_ros] shut the node down when reading the cards fails on ROS2
  On ROS2 the cards are read in a thread, and an exception there (e.g. a binary NDEF record,
  which record.data.decode() fails on) ended only the thread, and left the node running without
  reading the cards. The thread logs the error and shuts the node down, and the node exits with 1,
  as the exception ends the node on ROS1, so that on_exit of the launch file runs. Checked with a
  reader which fails on b'\xff'.decode(): the node logs the error and exits with 1. Pointed out by
  the review of Copilot.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01Xo6jRt2Rcht9sQS6ms6A4p
* [nfc_ros] README: ROS2
  How to build and run on ROS2 (the venv of nfcpy, demo.launch.xml), the
  error without the NFC device, and the upper case constant names of Tag.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01Xo6jRt2Rcht9sQS6ms6A4p
* [nfc_ros] add ROS2 launch file
  launch/demo.launch.xml: the ROS2 counterpart of demo.launch (on_exit=
  "shutdown" for required="true"), installed with the launch directory.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01Xo6jRt2Rcht9sQS6ms6A4p
* [nfc_ros] fail with an error message without the NFC reader
  Without a reader (or without the access to it), nfc.ContactlessFrontend()
  raises IOError(ENODEV), which ended the node with a traceback and an
  AttributeError in NFCInterface.__del_\_() (self.clf is not set then).
  Log "Failed to open the NFC reader: [Errno 19] No such device" and exit
  with 1 instead, on ROS1 and ROS2.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01Xo6jRt2Rcht9sQS6ms6A4p
* [nfc_ros] exit on SIGINT/SIGTERM on ROS2
  nfcpy's clf.connect() blocks until a tag is read, and returns False on
  a KeyboardInterrupt. On ROS1, rospy's signal handler shuts rospy down
  before raising it, so the loop of NFCROSNode.spin() ends; rclpy's signal
  handlers shut the context down only out of a spinning executor, so the
  node never exited on ROS2 (SIGTERM neither).
  - NFCInterface.read_card() takes nfcpy's `terminate` function, and
  NFCROSNode.spin() returns read_card() on shutdown with it,
  - on ROS2, __main_\_ reads the cards in a thread and spins the node with
  ros_compat.spin(), which returns on SIGINT/SIGTERM and shuts rclpy
  down, so that the thread returns; ROS1 is unchanged.
  Checked with nfcpy's 'udp' driver in place of the USB reader: the node
  exits with 0 on SIGINT and SIGTERM (it was killed with SIGKILL before).
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01Xo6jRt2Rcht9sQS6ms6A4p
* [nfc_ros] Felica tag types: set tag_type with the ROS2 constant names
  The FeliCa tags were never given their tag_type (`msg.tag_type ==
  Tag.NFC_TAG_Felica...` compared instead of assigning, so 0 was
  published), and those comparisons fail on ROS2 with an AttributeError, as
  the constants are upper cased there (NFC_TAG_FELICASTANDARD, ..., see
  CMakeLists.txt). Assign them, looking the constant up with
  tag_constant(), which falls back to the upper cased name on ROS2.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01Xo6jRt2Rcht9sQS6ms6A4p
* [nfc_ros] build on ROS2
  - CMakeLists.txt: the ROS2 build is prepended and the ROS1 original is
  wrapped in else()/endif() unchanged (only lines added):
  - the messages are generated with rosidl; rosidl only allows upper case
  constant names, so Tag.msg is generated from msg/Tag.msg with the
  constant names upper cased into the build directory
  (NFC_TAG_FelicaStandard -> NFC_TAG_FELICASTANDARD, ...), the other
  constants are already upper case,
  - rosidl_generate_interfaces() already installs the nfc_ros python
  package, so the modules of python/nfc_ros are installed into it,
  - nfcpy is not available from apt, and catkin_virtualenv has no ROS2
  port, so nfc_ros_node.py runs in a uv venv made from
  pyproject.toml/uv.lock (nfcpy==1.0.4, as requirements.txt) with
  jsk_ros1_ros2_compat_generate_uv_venv(), installed to share/ (so it is
  shipped in the deb); the script is installed with
  jsk_ros1_ros2_compat_install_python(USE_VENV).
  - package.xml: format 3, with $ROS_VERSION conditions on the ROS1/ROS2
  dependencies and build_type.
  - AMENT_IGNORE is removed.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01Xo6jRt2Rcht9sQS6ms6A4p
* [nfc_ros] rospy -> jsk_ros1_ros2_compat
  Use jsk_ros1_ros2_compat's rospy_rclpy_compat as `ros_compat` instead of
  rospy, so that the node runs on ROS1 and ROS2. `node` is None on ROS1.
  1. Mechanical part, generated without manual edits by
  cd nfc_ros
  FILES=$(grep -l '^import rospy$' node_scripts/*.py python/nfc_ros/*.py)
  sed -i '/^import rospy$/a from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat' $FILES
  python3 rospy_to_ros_compat.py $FILES
  rospy_to_ros_compat.py is the script of the ros_speech_recognition port
  (4c1d6ebc, "[ros_speech_recognition] rospy/actionlib/dynamic_reconfigure
  -> jsk_ros1_ros2_compat"): it converts the rospy calls whose ros_compat
  functions take the same arguments (log*, is_shutdown,
  Publisher -> create_publisher, init_node), adding the node argument
  (`node` in __main_\_ and in __init_\_ of NFCROSNode, which gets a
  `node=None` argument, `self.node` in its other methods), and removes
  `import rospy` when no rospy.xxx is left.
  2. By hand:
  - node_scripts/nfc_ros_node.py: NFCROSNode is given the node of
  ros_compat.init_node(), and is no longer assigned to `node`,
  - package.xml: exec_depend on jsk_ros1_ros2_compat (>= 3.0.2, the
  released jsk_common), on ROS1 too, as there is no fallback to rospy.
  The ROS1-only test/test_rospy_node.py keeps rospy.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01Xo6jRt2Rcht9sQS6ms6A4p
* Merge pull request `#584 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/584>`_ from k-okada/release-3.0.1
* Contributors: Kei Okada

3.0.1 (2026-11-10)
------------------
* Merge pull request #583 from k-okada/rosdistro-55247-review
  rosdistro#55247 review: rename power_switching_tools_ros to jsk_power_switching_tools, fix licenses
* use SPDX license identifiers (BSD-3-Clause, Apache-2.0) in package.xml
  BSD -> BSD-3-Clause and Apache / "Apache License, Version 2.0" ->
  Apache-2.0 in all packages (ros/rosdistro#55247 review). The LICENSE
  files of all of them are BSD 3-Clause and Apache 2.0 respectively.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* Contributors: Kei Okada

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
* Contributors: Kei Okada

2.1.31 (2025-05-13)
-------------------

2.1.30 (2025-05-10)
-------------------

2.1.29 (2025-01-05)
-------------------
* [nfc_ros] nfcpy requries python3.6+, skip kinetic which has python3.5 `#515 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/515>`_)
* Contributors: Kei Okada

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
* add test to check if ros node is loadable (`#463 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/463>`_)
* Add nfc_ros (`#406 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/406>`_)
* Contributors: Aoi Nakane, Kei Okada, Koki Shinjo, Naoto Tsukamoto, Shingo Kitagawa

2.1.24 (2021-07-26)
-------------------

2.1.23 (2021-07-21)
-------------------

2.1.22 (2021-06-10)
-------------------

2.1.21 (2020-08-19)
-------------------

2.1.20 (2020-08-07)
-------------------

2.1.19 (2020-07-21)
-------------------

2.1.18 (2020-07-20)
-------------------

2.1.17 (2020-04-16 21:51)
-------------------------

2.1.16 (2020-04-16 15:21)
-------------------------

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

2.0.11 (2015-10-07 14:16)
-------------------------

2.0.10 (2015-10-07 12:47)
-------------------------

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

2.0.1 (2015-06-19 21:21)
------------------------

2.0.0 (2015-06-19 10:41)
------------------------

1.0.71 (2015-05-17)
-------------------

1.0.70 (2015-05-08)
-------------------

1.0.69 (2015-05-05 12:28)
-------------------------

1.0.68 (2015-05-05 09:49)
-------------------------

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

1.0.60 (2015-02-03 10:12)
-------------------------

1.0.59 (2015-02-03 04:05)
-------------------------

1.0.58 (2015-01-07)
-------------------

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

1.0.51 (2014-10-20 16:01)
-------------------------

1.0.50 (2014-10-20 01:50)
-------------------------

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

1.0.44 (2014-09-26 09:17)
-------------------------

1.0.43 (2014-09-26 01:08)
-------------------------

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

1.0.37 (2014-09-08)
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

1.0.19 (2014-05-06)
-------------------

1.0.18 (2014-05-04)
-------------------

1.0.17 (2014-04-20)
-------------------

1.0.16 (2014-04-19 23:29)
-------------------------

1.0.15 (2014-04-19 20:19)
-------------------------

1.0.14 (2014-04-19 12:52)
-------------------------

1.0.13 (2014-04-19 11:06)
-------------------------

1.0.12 (2014-04-18 16:58)
-------------------------

1.0.11 (2014-04-18 08:18)
-------------------------

1.0.10 (2014-04-17)
-------------------

1.0.9 (2014-04-12)
------------------

1.0.8 (2014-04-11)
------------------

1.0.7 (2014-04-10)
------------------

1.0.6 (2014-04-07)
------------------

1.0.5 (2014-03-31)
------------------

1.0.4 (2014-03-29)
------------------

1.0.3 (2014-03-19)
------------------

1.0.2 (2014-03-12)
------------------

1.0.1 (2014-03-07)
------------------

1.0.0 (2014-03-05)
------------------
