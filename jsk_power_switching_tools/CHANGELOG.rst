^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_power_switching_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge pull request #583 from k-okada/rosdistro-55247-review
  rosdistro#55247 review: rename power_switching_tools_ros to jsk_power_switching_tools, fix licenses
* use SPDX license identifiers (BSD-3-Clause, Apache-2.0) in package.xml
  BSD -> BSD-3-Clause and Apache / "Apache License, Version 2.0" ->
  Apache-2.0 in all packages (ros/rosdistro#55247 review). The LICENSE
  files of all of them are BSD 3-Clause and Apache 2.0 respectively.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* jsk_power_switching_tools: add LICENSE (BSD-3-Clause)
  package.xml declares BSD, but there was no license text in the package
  (ros/rosdistro#55247 review). Same BSD 3-Clause text as the other
  packages of this repository.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* power_switching_tools_ros: rename to jsk_power_switching_tools (REP-144)
  The ROS2 release review of jsk_3rdparty in ros/rosdistro#55247 says
  (https://github.com/ros/rosdistro/pull/55247#issuecomment-6757867277):
  > - [ ] Each package meets REP-144 naming conventions.
  >   - The 11 names other than `power_switching_tools_ros` have been
  >     released in noetic (2.1.31-4), so they're grandfathered.
  >   - `power_switching_tools_ros` is new. The `_ros` suffix is for a
  >     library wrapper; `jsk_power_switching_tools` would be better.
  It has never been released (not in noetic either), so rename it now,
  before anything depends on a released name: the package, the python
  module (power_switching_tools_ros -> jsk_power_switching_tools) and the
  references in the launch files, node scripts, udev script, README and
  the CHANGELOG title. Older CHANGELOG entries are kept as they are.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* Merge pull request #571 from pazeshun/handle-slow-power-switch
  [power_switching_tools_ros] Handle cases where power switching tool is slow
* [power_switching_tools_ros] Handle cases where power switching tool is slow
  Response from the power switching tool is very slow in some PCs (e.g., PC with many USB devices).
  In that case, a message of the monitored topic comes before the first power cycle when init_with_power_cycle is true.
  Previous implementation gets that message and wrongly considers that it came after the first power cycle.
  This causes unnecessary power cycle because a next message will not come just after the power cycle and that interval is considered a communication failure.
  This commit cuts this waste by stopping subscribing the monitored topic before the first power cycle.
* Contributors: Kei Okada, Shun Hasegawa

3.0.0 (2026-11-06)
------------------
* [voicevox, chaplus_ros, emotion_analyzer, gdrive_ros, google_chat_ros, influxdb_store, power_switching_tools_ros, respeaker_ros, ros_speech_recognition, rospatlite, sesame_ros, switchbot_ros] depend on ros_environment for ROS2 buildfarm
  The buildfarm builds each package with only its own build dependencies,
  so without ros_environment $ROS_VERSION is empty and CMakeLists.txt takes
  the ROS1 branch (same as jsk_common#1835). voicevox also guesses
  ROS_VERSION from AMENT_PREFIX_PATH when it is unset.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [power_switching_tools_ros] add ROS2 support
  Single package.xml/CMakeLists.txt branched on $ROS_VERSION, same as
  chaplus_ros and jsk_common's ROS2-ported packages.
  - CMakeLists.txt: the ROS2 block is prepended and the ROS1 original is
  wrapped in else()/endif() unchanged (no re-indentation). The python
  package is installed with ament_python_install_package(), the driver
  scripts go to lib/ with jsk_ros1_ros2_compat_install_python() (python3
  shebang, jsk-ros-pkg/jsk_common#1832; a ROS2-only buildtool_depend
  >= 3.0.2), and scripts/ and udev/ to share/.
  - python: the original rospy code is kept as is for ROS1 users
  (PowerSwitchingDriverBase, UsbPppsHubDriver,
  UsbSerialTroubleshooterDriver and the __main__ blocks, used when
  jsk_ros1_ros2_compat, i.e. jsk_common >= 3.0, is not available), and
  PowerSwitchingDriverBaseCompat, UsbPppsHubDriverCompat and
  UsbSerialTroubleshooterDriverCompat are added next to them as the same
  code written with jsk_ros1_ros2_compat's rospy_rclpy_compat
  (init_node, spin, get_param, create_service, get_response_class, sleep,
  ``log*``, logfatal, get_name, signal_shutdown;
  jsk-ros-pkg/jsk_common#1831, >= 3.0.2), which run on both ROS1 and
  ROS2 when jsk_ros1_ros2_compat is available. Only the imports (rospy
  and std_srvs' ``*Response`` classes do not exist on ROS2) and the __main__
  conditions of the original lines change. The compat version of the
  ~hub_location/~hub_port check also treats None as missing, since
  ros_compat.get_param() returns the default instead of raising KeyError.
  - auto_power_cycle.py and the launch files stay ROS1-only for now: it
  subscribes with rospy.AnyMsg and calls the power cycle service
  synchronously from a timer callback, which would deadlock a
  single-threaded rclpy executor.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [power_switching_tools_ros] Add webcam problem to README
* [power_switching_tools_ros] Add how to try latest uhubctl
* [power_switching_tools_ros] Enable to set multiple hub_location and hub_port
* [power_switching_tools_ros] Fix udev setting script
* [power_switching_tools_ros] Add more document of power switching test
* [power_switching_tools_ros] Enable to add supplementary options to uhubctl and specify path to uhubctl
* [power_switching_tools_ros] Add another advantage of USB PPPS hubs
* [power_switching_tools_ros] Enable to prepare multiple udev rules for USB-Serial troubleshooters
* [power_switching_tools_ros] Support USB PPPS hubs
* [power_switching_tools_ros] Make it easier to support new device
* century_ros -> power_switching_tools_ros
* Contributors: Kei Okada, Shun Hasegawa
