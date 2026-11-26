^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package emotion_analyzer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.2 (2026-11-26)
------------------
* Merge pull request `#602 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/602>`_ from k-okada/license-bundled-components
  Declare the licenses of bundled components (hume, pixel-ring, Vosk models)
* emotion_analyzer: note the license of hume bundled in the venv
  PyPI says hume 0.6.0 is "Other/Proprietary License", but the LICENSE file of
  its sdist is the MIT License. The scripts use HumeStreamClient, which was
  removed in hume 0.7, so keep 0.6.0 and note it in LICENSE.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01Xo6jRt2Rcht9sQS6ms6A4p
* Merge pull request `#594 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/594>`_ from k-okada/fix-error-handling
* emotion_analyzer: treat an empty Hume API key as missing in text service
  analyze_text_service.py checked `api_key is None`, but emotion_analyzer
  launch files always set hume_api_key from api_key, whose default is "".
  So the text service started without a key and only failed later on each
  request to the Hume API, while analyze_audio_service.py already refused
  an empty key with `not api_key`.
  Use the same `not api_key` check and say how to give the key. A node
  started with a key behaves as before; without a key it now stops at
  start-up with a clear message, as the audio service already does.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01Xo6jRt2Rcht9sQS6ms6A4p
* Merge pull request `#595 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/595>`_ from k-okada/readme-update
  Update the READMEs for ROS 1 / ROS 2, with badges and demos
* emotion_analyzer: update README.md for ROS 1 / ROS 2
  Describe how to run it on ROS 2 and ROS 1, its interface (topics,
  services, actions, parameters) as in the source, and its requirements.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01Xo6jRt2Rcht9sQS6ms6A4p
* Merge branch 'master' into speech-recognition-drop-phrase-during-tts
* Merge pull request `#592 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/592>`_ from k-okada/fix-downward-url
* emotion_analyzer: pin hume of requirements.in (all the versions on PyPI are yanked)
  All the versions of hume on PyPI are yanked now, and pip installs a yanked version
  only when it is pinned by ==, so that catkin_virtualenv (pip-compile of
  requirements.in) fails with "Could not find a version that satisfies the requirement
  hume[stream]" (e.g. ROS One on the buildfarm of ubi-agni and ROS-O of the CI). Pin
  0.6.0, the version of pyproject.toml / uv.lock of ROS2 (the legacy HumeStreamClient
  of the scripts is removed in 0.7; 0.14.1, which the CI resolved before, has no
  "stream"), for python 3.9 and later, and 0.4.2 for python 3.8 (noetic).
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01Xo6jRt2Rcht9sQS6ms6A4p
  (cherry picked from commit 9a6bf6c761a92aa9a9d2e23a70193b3295d7347c)
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
* Merge pull request #581 from k-okada/ros2-buildfarm-ros-environment
  Fix ROS2 buildfarm binary jobs: ros_environment, bundled flac, numpy wheel
* [emotion_analyzer] use python3-soundfile from apt instead of the uv venv
  The numpy wheel pulled by soundfile fails dh_strip of the buildfarm binary
  job (libscipy_openblas64\_*.so). python3-soundfile is already an
  exec_depend and the venv sees the system site-packages.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
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
* [emotion_analyzer] add ROS2 support
  Single package.xml/CMakeLists.txt branched on $ROS_VERSION, same as
  chaplus_ros and jsk_common's ROS2-ported packages.
  - CMakeLists.txt: the ROS2 block is prepended and the ROS1 original is
  wrapped in else()/endif() unchanged. ROS2 generates the services with
  rosidl_generate_interfaces(), which already installs the
  emotion_analyzer python package, so src/emotion_analyzer/utils is
  installed into it instead of a second ament_python_install_package().
  - package.xml: format 3 for conditions; audio_common_msgs (imported by
  the scripts but only pulled in by audio_capture before) is listed
  explicitly. hume has no rosdep key and catkin_virtualenv has no ROS2
  port, so ROS2 users pip install it (README.md).
  - analyze\_{text,audio}_service.py, utils/audio_buffer.py: use
  jsk_ros1_ros2_compat's rospy_rclpy_compat, falling back to rospy when
  it is not importable (jsk_common < 3.0, ROS1 only).
  - launch/emotion_analyzer.launch.xml: ROS2 version of
  emotion_analyzer.launch.
  - Catch exceptions from the Hume request and return {"error": ...}
  (same format as the existing no-audio case): rospy only returned a
  service error, but an exception in a service callback kills the rclpy
  node.
  - hume (not in apt; catkin_virtualenv has no ROS2 port) is installed
  into a uv venv made from pyproject.toml/uv.lock at build time with
  jsk_ros1_ros2_compat_generate_uv_venv() (jsk-ros-pkg/jsk_common#1831),
  and both services are installed with
  jsk_ros1_ros2_compat_install_python(... USE_VENV) to run with it,
  instead of asking users to `pip install hume[stream]`. hume is pinned
  below 0.7, since the scripts use the legacy HumeStreamClient that 0.7
  removed (the latest hume, 0.14, can not run them on ROS1 either).
  soundfile is in the venv too, as the audio service runs with the
  venv's python.
  - package.xml: ROS2-only buildtool_depend on jsk_ros1_ros2_compat
  (>= 3.0.2).
  - launch/sample_emotion_analyzer.launch.xml: ROS2 version of
  sample_emotion_analyzer.launch, with audio_capture's
  capture_wave.launch.xml.
  - main: ros_compat.init_node()/spin() instead of the ROS_VERSION branch.
  - AnalyzeTextResponse/AnalyzeAudioResponse via
  ros_compat.get_response_class() instead of a ROS_VERSION import branch.
  - audio_buffer.py: no explicit private_name() (create_subscription()
  maps ~foo itself now).
  Uses jsk-ros-pkg/jsk_common#1831 (>= 3.0.2 on ROS2); the ROS1 fallback
  forwards the new functions to rospy.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* Contributors: Kei Okada

2.1.31 (2025-05-13)
-------------------
* add LICENSE files
* Contributors: Kei Okada

2.1.30 (2025-05-10)
-------------------
* Add Emotion Analyzer (`#527 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/527>`_)
* Contributors: Ayaha Nagata, Yoshiki Obinata

