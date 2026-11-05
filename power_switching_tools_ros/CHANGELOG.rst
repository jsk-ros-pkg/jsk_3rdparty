^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package power_switching_tools_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
