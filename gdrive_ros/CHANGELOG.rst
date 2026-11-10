^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gdrive_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
* [gdrive_ros] add ROS2 support
  Single package.xml/CMakeLists.txt branched on $ROS_VERSION, same as
  chaplus_ros and jsk_common's ROS2-ported packages.
  - CMakeLists.txt: the ROS2 block is prepended and the ROS1 original is
  wrapped in else()/endif() unchanged. ROS2 generates the services with
  rosidl_generate_interfaces(), which already installs the gdrive_ros
  python package, so gdrive_ros_client.py is installed into it. The
  installed ROS2 copies of the node scripts get a python3 shebang.
  - package.xml: format 3 for conditions. catkin_virtualenv has no ROS2
  port, so ROS2 gets PyDrive from the pydrive-pip rosdep key.
  - gdrive_server_node.py, sample_gdrive_rospy_client.py: use
  jsk_ros1_ros2_compat's rospy_rclpy_compat, falling back to rospy when
  it is not importable (jsk_common < 3.0, ROS1 only). ~upload and
  ~upload_multi keep their /gdrive_server/... names on ROS2.
  - gdrive_ros_client.py: GDriveROSClient takes an optional rclpy node;
  on ROS2 it calls the services through ros_compat.call_service(). The
  ROS1 code path is unchanged.
  - launch/gdrive_server.launch.xml: ROS2 version of gdrive_server.launch.
  ``sample/*.launch`` and the euslisp client stay ROS1-only.
  - The ROS2 scripts are installed with jsk_ros1_ros2_compat's
  jsk_ros1_ros2_compat_install_python() (python3 shebang), from
  jsk-ros-pkg/jsk_common#1832, as a ROS2-only buildtool_depend
  (>= 3.0.2) that stays off rosidl's build dependencies.
  - PyDrive (not in apt; catkin_virtualenv has no ROS2 port) is installed
  into a uv venv made from pyproject.toml/uv.lock at build time with
  jsk_ros1_ros2_compat_generate_uv_venv() (jsk-ros-pkg/jsk_common#1831),
  which gdrive_server_node.py runs with. This also avoids a pip rosdep
  key in package.xml, which bloom can not release. pyOpenSSL is in the
  venv too, since oauth2client imports it when present and the system
  one does not match the venv's cryptography.
  - gdrive_server_node.py / sample_gdrive_rospy_client.py: main with
  ros_compat.init_node()/spin(); UploadResponse/MultipleUploadResponse
  via ros_compat.get_response_class(); no explicit private_name().
  - gdrive_ros_client.py: service calls go through ros_compat.call_service()
  on both ROS1 and ROS2 instead of a ROS_VERSION branch that used
  ros_compat only on ROS2 (the ROS1 fallback forwards to
  rospy.ServiceProxy). wait_for_gdrive_server() uses
  ros_compat.wait_for_service().
  Uses jsk-ros-pkg/jsk_common#1831 (>= 3.0.2 on ROS2).
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* Merge branch 'master' into power_switching_tools_ros
* Merge pull request #555 from k-okada/fix_gdrive_url
  gdrive_server_node.py: update url format.
* gdrive_server_node.py: update url format. https://drive.google.com/uc?id={} is user content end p oint and actual url is https://drive.usercontent.google.com/download?id={}&authuser=0
* Contributors: Kei, Kei Okada, Shun Hasegawa

2.1.31 (2025-05-13)
-------------------

2.1.30 (2025-05-10)
-------------------

2.1.29 (2025-01-05)
-------------------

2.1.28 (2023-07-24)
-------------------
* use query with folder + title, instead of list all files then apply filters (`#481 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/481>`_)
* Contributors: Kei Okada

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
* gdrive_ros: use virtualenv (`#458 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/458>`_)

  * use python3 for pydrive_ros
    latest pydrive requires newer rsa??
  * gdrive_ros: catkin_virtualenv changes mode to 100644
  * gdrive_ros: enable to use relative path for settings_yaml
  * gdrive_ros: use catkin_virtualenv

* [gdrive_ros] fix sys.version error (`#429 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/429>`_)
* [gdrive_ros] Add ros client library and examples ( for roseus and rospy) (`#295 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/295>`_)

  * euslisp directory is installed via 'install(DIRECTORY euslisp', so we can remove 'install(PROGRAMS euslisp/sample-gdrive-ros-client.l'
  * [gdrive_ros] fix module import
  * [gdrive_ros] add rospy client and update examples
  * [gdrive_ros] update CMakeLists.txt to add installation of euslisp and sample
  * [gdrive_ros] Add a roseus client library and sample
  * [gdrive_ros] add gdrive_ros client

* [gdrive_ros] add machine arg in gdrive_server.launch (`#288 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/288>`_)

  * add machine arg in gdrive_server.launch

* Contributors: Aoi Nakane, Kei Okada, Koki Shinjo, Naoto Tsukamoto, Shingo Kitagawa

2.1.24 (2021-07-26)
-------------------

2.1.23 (2021-07-21)
-------------------
* add rostest in test_depend (`#263 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/263>`_)
* fix-typo-gdrive (`#272 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/272>`_)
* add version check at top of gdrive_server_node (`#270 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/270>`_)

* Contributors: Koki Shinjo, Shingo Kitagawa

2.1.22 (2021-06-10)
-------------------
* set max authentication trial times as infinite (`#249 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/249>`_)
* catch ServerNotFoundError in gdrive_server_node.py ((`#240 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/240>`_)
* update doc links in gdrive_ros (`#233 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/233>`_)
* [gdrive_ros] add node_name arg in gdrive_server.launch (`#234 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/234>`_)
* add roslaunch test (`#232 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/232>`_)
* [gdrive_ros] add argument respawn (`#230 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/230>`_)

* Contributors: Shingo Kitagawa

2.1.21 (2020-08-19)
-------------------

2.1.20 (2020-08-07)
-------------------

2.1.19 (2020-07-21)
-------------------

2.1.18 (2020-07-20)
-------------------
* [gdrive_ros] Fix gdrive_ros to catch error properly (`#205 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/205>`_)

  * fix ros log format in gdrive_ros
  * add oserr when file not found in gdrive_ros

* Contributors: Kei Okada, Shingo Kitagawa

2.1.17 (2020-04-16)
-------------------

2.1.16 (2020-04-16)
-------------------

2.1.15 (2019-12-12)
-------------------
* add gdrive_ros package (`#182 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/182>`_)
* Contributors: Shingo Kitagawa
