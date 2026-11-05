^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package google_chat_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.0 (2026-11-06)
------------------
* Merge pull request #581 from k-okada/ros2-buildfarm-ros-environment
  Fix ROS2 buildfarm binary jobs: ros_environment, bundled flac, numpy wheel
* [voicevox, chaplus_ros, emotion_analyzer, gdrive_ros, google_chat_ros, influxdb_store, power_switching_tools_ros, respeaker_ros, ros_speech_recognition, rospatlite, sesame_ros, switchbot_ros] depend on ros_environment for ROS2 buildfarm
  The buildfarm builds each package with only its own build dependencies,
  so without ros_environment $ROS_VERSION is empty and CMakeLists.txt takes
  the ROS1 branch (same as jsk_common#1835). voicevox also guesses
  ROS_VERSION from AMENT_PREFIX_PATH when it is unset.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* Merge branch 'master' into fix_hoya
* Merge branch 'master' into voicevox-ros1-ros2
* AMENT_IGNORE: do not add AMENT_IGNORE to packages already ported to ROS2
  chaplus_ros, emotion_analyzer, gdrive_ros, google_chat_ros,
  influxdb_store, power_switching_tools_ros, respeaker_ros, rospatlite,
  sesame_ros and switchbot_ros are already ROS2 packages on master, so
  this PR must not AMENT_IGNORE them. Also remove
  ros_speech_recognition/AMENT_IGNORE, as this PR ports it to ROS2.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* Merge branch 'master' into ros2-speech-recognition-only
* Merge pull request #575 from k-okada/ros2-support
  Add ROS2 (jazzy) support to chaplus_ros, emotion_analyzer, gdrive_ros, google_chat_ros, influxdb_store, power_switching_tools_ros, respeaker_ros, rospatlite, sesame_ros, switchbot_ros
* AMENT_IGNORE packages ported to ROS2 in jsk-ros-pkg/jsk_3rdparty#575
  chaplus_ros, emotion_analyzer, gdrive_ros, google_chat_ros,
  influxdb_store, power_switching_tools_ros, rospatlite, sesame_ros and
  switchbot_ros are ROS1 only here, so the ROS2 CI skips them until #575.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [google_chat_ros] add ROS2 support
  Single package.xml/CMakeLists.txt branched on $ROS_VERSION, same as
  jsk_common's ROS2-ported packages.
  - CMakeLists.txt: the ROS2 block is prepended and the ROS1 original is
  wrapped in else()/endif() unchanged (no re-indentation). ROS2
  generates msg/*.msg and SendMessage.action with rosidl, and installs
  src/google_chat_ros into the python package that
  rosidl_generate_interfaces() already installs.
  - package.xml: on ROS2 the python dependencies come from rosdep keys
  (python3-googleapi, python3-oauth2client, python3-google-cloud-pubsub-pip,
  python3-gdown-pip, ...) instead of catkin_virtualenv.
  - google_chat.py: its free-standing rospy.log*() calls go through
  jsk_ros1_ros2_compat's log_utils (imported as `rospy`). Also fall back
  to ssl.SSLContext when ssl.wrap_socket() is missing (removed in Python
  3.12), so the url receiving mode can start on noble/jazzy.
  - google_chat_ros_node.py: uses rospy_rclpy_compat (get_param, log*,
  create_publisher, Rate), keeping the ROS1 structure as-is. The send
  action server uses a small actionlib.SimpleActionServer-like wrapper
  of rclpy's ActionServer, so rest_cb() is shared. GoogleChatROS() blocks
  in the url/pubsub receiving modes (rospy spins in its own threads), so
  on ROS2 the node is spun in a background thread; rospy.on_shutdown()
  becomes the rclpy context's on_shutdown().
  - jsk_ros1_ros2_compat is only released with jsk_common >= 3.0 (not
  in noetic/ROS-O yet), so it is a ROS2-only dependency and the scripts
  fall back to rospy when it is not importable.
  - ROS2 python dependencies: a uv venv made from pyproject.toml/uv.lock
  (the same versions as requirements.in.python3.12) with
  jsk_ros1_ros2_compat_generate_uv_venv() (jsk-ros-pkg/jsk_common#1831),
  in place of rosdep keys, two of which were pip keys that bloom can not
  release. google_chat_ros_node.py runs with the venv's python3
  (install_python(... USE_VENV)); helper.py needs none of them.
  oauth2client picks up the system pyOpenSSL together with the system
  cryptography here (neither is in the venv), so they match.
  - helper.py is ported too: create_simple_action_client() for
  google_chat_ros/send, dialogflow_client/text_action and robotsound_jp
  (sound_play_msgs on ROS2), get_param/create_subscription. On ROS2 the
  subscription callback only queues the message and a worker thread
  sends the goals, since waiting for an action result in an executor
  callback would block the executor. dialogflow_task_executive is not
  ported to ROS2, so to_dialogflow_client is skipped with a warning when
  its messages can not be imported; it is a ROS1-only dependency, and so
  is the unused DialogResponse import of the node.
  - google_chat_ros_node.py: create_simple_action_server()/on_shutdown()
  from jsk_ros1_ros2_compat replace the local action
  server wrapper and shutdown hook. Image upload via gdrive_ros now also
  works on ROS2 (a client in a reentrant callback group, since it is
  called from the action execute callback). GoogleChatROS(), which blocks
  in url/pubsub mode, runs in a daemon thread and the node spins in the
  main thread: rclpy did not shut down on SIGINT/SIGTERM while spinning in
  a non-main thread.
  - google_chat.py: GoogleChatHTTPSServer.run() wakes up every second
  (httpd.timeout), so kill() stops the url mode server on shutdown
  without waiting for a request; on ROS1 the node did not exit on
  rosnode kill either.
  - launch/google_chat.launch.xml: ROS2 version of google_chat.launch;
  yaml_file is loaded with <param from>, reading launch/empty_params.yaml
  unless use_yaml, since <param from> takes no if=.
  ROS1-only: the euslisp client and the rostests.
  - google_chat_ros_node.py: main with ros_compat.init_node()/spin()
  (MultiThreadedExecutor on ROS2, created after init); GoogleChatROS()
  runs in a daemon thread on both ROS versions. The ROS2-only
  _ros2_upload_file() uses the node's own logger instead of ros_compat,
  so that ROS_VERSION branches use plain rospy/rclpy only.
  - helper.py: the message queue + worker thread is used on both ROS
  versions, so __init_\_ and main need no ROS_VERSION branch; main with
  ros_compat.init_node()/spin().
  Uses jsk-ros-pkg/jsk_common#1831 (>= 3.0.2 on ROS2).
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* Merge branch 'master' into power_switching_tools_ros
* Merge pull request #568 from k-okada/k-okada/add_ga_test
  Enable CI test runs and fix a few test failures
* [google_chat_ros] bump gdown to 4.7.3 to drop pkg_resources dependency
  gdown 4.4.0 imports pkg_resources at module load time, which breaks
  under Python 3.12 venvs where catkin_virtualenv preinstalls a
  setuptools version that no longer ships pkg_resources. gdown>=4.5
  reads its own version as a plain string instead, so this needs no
  compat shim. gdown.download()'s url/output/id signature is unchanged.
* Contributors: Kei Okada, Shun Hasegawa

2.1.31 (2025-05-13)
-------------------

2.1.30 (2025-05-10)
-------------------
* google_chat_ros: enable to get therad_name message result (`#516 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/516>`_)
* Add ROS-O 24.04 test (`#521 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/521>`_)
* Contributors: Kei Okada, Yoshiki Obinata

2.1.29 (2025-01-05)
-------------------
* [google_chat_ros] check whether google_chat_button's parameters include NoneType object for avoiding rospy serialization error (`#501 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/501>`_)
* [google_chat_ros] Fix an error message typo; https ==> url (`#492 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/492>`_)
* Contributors: Kei Okada, Naoto Tsukamoto

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
* google_chat_ros_node.py: display project_id, subscription_id (`#459 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/459>`_)
* add test to check if ros node is loadable, (`#463 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/463>`_)

  * install python files under CATKIN_PACKAGE_BIN_DESTINATION
  * google_chat_ros/test_rospy_node.py: skip healper.py
    helper.py depends on dialogflow_task_executive. However, when we add this to the <depend> of package.xml, it appempts to build venv using 'dialogflow_task_executive/requirements.txt'. This requires having the same PYTHON_INTERPRETER for both dialogflow and chat ros package. The issue is that dialogflow_task_executive heavily relies on system Python modules, including ROS, making it difficult to use dialogflow with Python3 on Melodic
  * add catkin_install_python for test, it is also change installed directory from BIN to SHARE, because we want to have same directory structure between devel and install
  * add test to check if ros node is loadable
    If we use python2 PYTHON_INTERPRETER on 20.04, python2 fails to load rospy in /opt/ros/noetic, because rospy moduels are alraedy updated.
    If we use python3 PYTHON_INTERPRETER on 18.04, python3 can load rospy in /opt/ros/melodic by chance.
    c.f. https://github.com/jsk-ros-pkg/jsk_3rdparty/pull/367

* google_chat_ros/README.md: add troubleshooting (`#450 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/450>`_)
* fix google_chat_ros noetic build errors (`#422 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/422>`_)
* use same timestamp for one goal (`#403 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/403>`_)
  * save google chat image in /chat_notification with timestamp

* add google_chat_ros (`#392 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/392>`_)

* Contributors: Aoi Nakane, Kei Okada, Naoto Tsukamoto, Shingo Kitagawa, Yoshiki Obinata

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
