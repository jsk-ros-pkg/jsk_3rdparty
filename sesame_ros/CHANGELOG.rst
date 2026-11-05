^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sesame_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
* [sesame_ros] add ROS2 support
  Single package.xml/CMakeLists.txt branched on $ROS_VERSION, same as
  chaplus_ros and jsk_common's ROS2-ported packages.
  - CMakeLists.txt: the ROS2 block is prepended and the ROS1 original is
  wrapped in else()/endif() unchanged. ROS2 generates ``srv/*.srv`` with
  rosidl_generate_interfaces() and installs sesame_server.py /
  list_sesames.py with a python3 shebang. ROS2 uses the system
  python3-requests instead of catkin_virtualenv (ROS1 only).
  - package.xml: format 2 -> 3 (needed for condition=).
  - sesame_server.py: uses jsk_ros1_ros2_compat's rospy_rclpy_compat
  (get_param, create_service, create_response, ``log*``), with the same
  rospy-forwarding fallback as chaplus_ros when jsk_ros1_ros2_compat
  (jsk_common >= 3.0) is not available. The original
  ``StatusResponse(*values)``/``CommandResponse(*values)`` lines are kept, with
  the response classes from ros_compat.get_response_class() (ROS2
  responses take neither positional arguments nor None otherwise).
  - test/test_rospy_node.test (rostest) stays ROS1-only.
  - The ROS2 scripts are installed with jsk_ros1_ros2_compat's
  jsk_ros1_ros2_compat_install_python() (python3 shebang), from
  jsk-ros-pkg/jsk_common#1832, as a ROS2-only buildtool_depend
  (>= 3.0.2) that stays off rosidl's build dependencies.
  Drop make_response(): StatusResponse/CommandResponse come from
  ros_compat.get_response_class(), which also takes positional fields on
  ROS2, so the original `return StatusResponse(*self._get_sesame_status())`
  lines are kept. main with ros_compat.init_node()/spin(), no explicit
  private_name(). Uses jsk-ros-pkg/jsk_common#1831 (>= 3.0.2 on ROS2).
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* Contributors: Kei Okada

2.1.31 (2025-05-13)
-------------------

2.1.30 (2025-05-10)
-------------------
* CI: add ROS-O testing on arm (`#528 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/528>`_), fix sesame_ros on arm64
  * CI: add ROS-O testing on arm
  * CI: use ros-one-catkin-virtualenv
  * sesami_ros: add requirements.in.python3.12, for arm64 22.04/24.04
* Contributors: Kei Okada, Yoshiki Obinata

2.1.29 (2025-01-05)
-------------------
* Support ros-o / Ubuntu 22.04 (`#512 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/512>`_)

  * [sesame_ros] relax venv check
  * [sesame_ros] remove cffi which cannot be installed in Ubuntu 22.04 env & not used in sesame_ros

* Contributors: Yoshiki Obinata

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
* Explicit python interpreter in catkin_virtualenv (`#367 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/367>`_)
* Contributors: Kei Okada, Shingo Kitagawa, Yoshiki Obinata

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
* Fix for noetic (`#200 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/200>`_)

  * update sesame_ros/requirements.txt to pass https://github.com/locusrobotics/catkin_virtualenv/blob/master/README.md#locking-dependencies

* Contributors: Kei Okada

2.1.17 (2020-04-16)
-------------------
* update pip modules for security reason (`#196 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/196>`_)
* add idna==2.7
  to cloes ERROR: requests 2.20.0 has requirement idna<2.8,>=2.5, but you'll have idna 2.9 which is incompatible.
* Bump urllib3 from 1.22 to 1.24.2 in /sesame_ros
  Bumps [urllib3](https://github.com/urllib3/urllib3) from 1.22 to 1.24.2.
  - [Release notes](https://github.com/urllib3/urllib3/releases)
  - [Changelog](https://github.com/urllib3/urllib3/blob/master/CHANGES.rst)
  - [Commits](https://github.com/urllib3/urllib3/compare/1.22...1.24.2)
  Signed-off-by: dependabot[bot] <support@github.com>
* Bump requests from 2.19.1 to 2.20.0 in /sesame_ros
  Bumps [requests](https://github.com/psf/requests) from 2.19.1 to 2.20.0.
  - [Release notes](https://github.com/psf/requests/releases)
  - [Changelog](https://github.com/psf/requests/blob/master/HISTORY.md)
  - [Commits](https://github.com/psf/requests/compare/v2.19.1...v2.20.0)
  Signed-off-by: dependabot[bot] <support@github.com>
* Bump pyopenssl from 16.2.0 to 17.5.0 in /sesame_ros
  Bumps [pyopenssl](https://github.com/pyca/pyopenssl) from 16.2.0 to 17.5.0.
  - [Release notes](https://github.com/pyca/pyopenssl/releases)
  - [Changelog](https://github.com/pyca/pyopenssl/blob/master/CHANGELOG.rst)
  - [Commits](https://github.com/pyca/pyopenssl/compare/16.2.0...17.5.0)
  Signed-off-by: dependabot[bot] <support@github.com>
* Contributors: Kei Okada, dependabot[bot]

2.1.16 (2020-04-16)
-------------------
* add i386 tests (`#191 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/191>`_)
* specify python module number, closes `#190 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/190>`_
* Contributors: Kei Okada

2.1.15 (2019-12-12)
-------------------
* [sesame_ros] Add dependency for building cryptograpy (`#180 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/180>`_)

  * Add url of ROS wiki for sesame_ros
  * Add k-okada to maintainer of sesame_ros
  * Add libffi-dev and libssl-dev as dependencies of cryptography

* Contributors: Yuto Uchimi

2.1.14 (2019-11-21)
-------------------
* Add sesame_ros package (`#176 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/176>`_)
* Contributors: Yuto Uchimi
