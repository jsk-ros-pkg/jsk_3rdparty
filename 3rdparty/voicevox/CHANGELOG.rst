^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package voicevox
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

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
* Merge pull request #573 from Michi-Tsubaki/voicevox-ros1-ros2
  [voicevox] Support both ROS1 and ROS2 build
* Merge pull request #13 from k-okada/voicevox-string-soundrequest
  [voicevox] speak both SoundRequest and text (std_msgs/String) with output_type:=wav and audio
* [voicevox] README: speak SoundRequest and text (std_msgs/String)
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [voicevox] output_type:=wav: speak text (std_msgs/String) on robotsound_jp/text too
  sound_play only subscribes SoundRequest, so text_to_sound_request.py
  relays text (std_msgs/String) on robotsound_jp/text to sound_play as a
  SoundRequest to say (arg2: ~speaker), so that both output_type:=wav and
  output_type:=audio speak both SoundRequest on robotsound_jp and text on
  robotsound_jp/text. It uses rospy/rclpy directly, as
  jsk_ros1_ros2_compat is not a ROS1 dependency.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [voicevox] output_type:=audio: speak SoundRequest on robotsound_jp too
  request_synthesis.py --output-type audio also subscribes SoundRequest on
  robotsound (arg: text, arg2: voice, e.g. from SoundClient.say()) as
  sound_play does, so that clients of sound_play work with both
  output_type:=wav and output_type:=audio. Text (std_msgs/String) moves
  from robotsound_jp to robotsound_jp/text, as robotsound_jp is now
  SoundRequest.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* Merge pull request #12 from mqcmd196/fix/voicevox-audio-info
  Fix audio metadata for streamed VOICEVOX PCM
* Restore text subscription queue depth
* Remove redundant audio_info republish
* Fix audio metadata for streamed VOICEVOX PCM
* [voicevox] remove AMENT_IGNORE, as voicevox now supports ROS2
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* Merge branch 'master' into voicevox-ros1-ros2
* Merge pull request #575 from k-okada/ros2-support
  Add ROS2 (jazzy) support to chaplus_ros, emotion_analyzer, gdrive_ros, google_chat_ros, influxdb_store, power_switching_tools_ros, respeaker_ros, rospatlite, sesame_ros, switchbot_ros
* Merge pull request #11 from k-okada/voicevox-gpu-on-ros1-ros2
  [voicevox] Add use_gpu option and GPU warm-up (#577) on top of ROS1/ROS2 support
* [voicevox] Add use_gpu to the ROS2 launch file
  Same as voicevox_texttospeech.launch: pass use_gpu to
  run-voicevox-docker.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [voicevox] Warm up the VOICEVOX GPU engine after the docker container starts
* [voicevox] Add use_gpu option to select the GPU or CPU docker image
* Add ROS 2 command for listing speakers
  Updated command for listing speakers to include ROS 2.
  Co-authored-by: Copilot Autofix powered by AI <175728472+Copilot@users.noreply.github.com>
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
* [voicevox] Add volume_scale usage to README
* [voicevox] Depend on audio_common_msgs also in ROS1
* Merge pull request #10 from k-okada/voicevox-compat
  [voicevox] use jsk_ros1_ros2_compat, share request_synthesis.py, output_type:=wav|audio
* [voicevox] keep setuptools < 81 in the ROS1 virtualenv
  catkin_virtualenv (0.19.0 on ROS-O noble) preinstalls setuptools 82 for
  python >= 3.12, which no longer has pkg_resources, and pip-compile drops
  the setuptools pin of requirements_python3.12.txt from the generated
  requirements.txt. pyworld imports pkg_resources, so the engine
  (run-voicevox) failed with "No module named 'pkg_resources'". Install
  setuptools<81 into the devel/install venvs after
  voicevox_generate_virtualenv (python >= 3.12 only).
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [voicevox] wait for the engine before synthesizing
  Retry while the engine can not be reached (e.g. it is still starting up
  when the first text arrives) for up to $VOICEVOX_STARTUP_TIMEOUT
  seconds (300) every $VOICEVOX_RETRY_INTERVAL seconds (1), in both the
  speaker list request and the synthesis, so for both text2wave and
  --output-type audio; errors from the engine itself are not retried. As
  ros2_synthesize() in fb5a84cb did for the ROS2 path only. The --output-type audio
  callback logs a failed synthesis instead of dying.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [voicevox] publish the speech on audio from request_synthesis.py
  voicevox_audio.py synthesized the speech by itself (its own HTTP
  requests to the engine) only to change where the result goes: a wav file
  for sound_play (text2wave) vs raw PCM on an audio topic. Make that the
  only difference:
  - request_synthesis.py --output-type wav|audio. wav (default) writes the
  wav file as text2wave for sound_play, as before. audio runs a node
  (jsk_ros1_ros2_compat, so ROS1 and ROS2) that subscribes text
  (std_msgs/String) and publishes the speech as raw PCM (mono S16LE,
  ~sample_rate, ~volume_scale) on audio (audio_common_msgs/AudioData) in
  0.1 s chunks at playback speed, for e.g. audio_common's audio_play
  with format:=wave, possibly on another machine. ~speaker (id or name)
  is read for each message. The speaker lookup (get_speaker_id) and the
  cached synthesis (synthesize) are shared by both outputs; the cache
  key includes the sample rate and volume when they are given.
  - voicevox_texttospeech.launch(.xml): output_type:=wav|audio (wav by
  default) chooses between sound_play and this node (text on
  robotsound_jp), with sample_rate/volume_scale args; this replaces
  voicevox_audio.launch.xml. audio_info is dropped, as audio_play does
  not use it.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [voicevox] use jsk_ros1_ros2_compat, share request_synthesis.py between ROS1/ROS2
  - CMakeLists.txt: the ROS1/ROS2 branches only set what differs
  (SHARE_DESTINATION, VOICEVOX_TARGET_DIR, ENGINE_PATH, VENV_PATH, and how
  the venv is made); configure_file() of run-voicevox(-docker) and the
  install of text2wave, run-voicevox(-docker), launch and the engine are
  shared. The ROS1 values are the ones used before, so the ROS1 build is
  unchanged.
  - ROS2 venv: jsk_ros1_ros2_compat_generate_uv_venv() replaces the inline
  uv install/venv/sync/strip/install steps; uv comes pinned with
  jsk_ros1_ros2_compat (jsk-ros-pkg/jsk_common#1831). pyopenjtalk's
  dictionary is still downloaded at build time (g2p), as the installed
  venv is read-only. jsk_ros1_ros2_compat_install_python(... USE_VENV)
  installs request_synthesis.py/list_speakers.py with the venv's python3
  instead of the generated runpy wrappers.
  - request_synthesis.py: one code path for ROS1 and ROS2 instead of the
  separate ros2_synthesize(). It needs no ROS API any more: the ROS home
  is $ROS_HOME or ~/.ros (what rospkg.get_ros_home() returns), and the
  speaker list is cached in <ROS home>/voicevox/speakers.json on both,
  since this command runs for each utterance and ROS2 has no global
  parameter server for the rosparam voicevox/speakers cache.
  - voicevox_audio.py: ros_compat.spin_and_shutdown().
  - package.xml: jsk_ros1_ros2_compat (>= 3.0.2) for ROS2.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [voicevox] fix duplicate unless attribute in voicevox_texttospeech.launch
  The voicevox_server node had both unless="$(arg use_docker)" and the
  unless="$(eval ...)" added in 5a483c2d, which already covers
  use_docker. roslaunch parses launch files with xml.dom.minidom, which
  rejects a duplicate attribute, so the launch file could not be loaded.
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
* Use /audio (wav)
* Stabilize voice synthesis
* [voicevox] Install uv venv to fix ROS2 deb package
  - Create the venv in the build directory with `uv venv --relocatable`
  and install it to share/voicevox/venv, so that the deb does not
  depend on the source tree of the build machine.
  - Install uv with UV_UNMANAGED_INSTALL so that it does not write to
  $HOME, and use UV_CACHE_DIR in the build directory.
  - Strip .so files in the venv in advance, because dh_strip fails on
  some prebuilt wheels (e.g. numpy lapack_lite).
  - Download the pyopenjtalk dictionary at build time, because the
  installed venv is not writable by users.
  - Add python3-dev to build_depend to build pyopenjtalk and pyworld.
* [voicevox] Support both ROS1 and ROS2 build
  - Switch build type by $ROS_VERSION in package.xml and CMakeLists.txt
  (same style as jsk_common_msgs). ROS1 build is kept as is.
  - ROS2: create virtualenv with uv (pyproject.toml / uv.lock)
  - Add launch/voicevox_texttospeech.launch.xml for ROS2 with the same
  arguments as voicevox_texttospeech.launch
  - text2wave, run-voicevox, request_synthesis.py: use ros2 / file cache
  when ROS_VERSION=2
  - Update README for ROS2
* Merge pull request #570 from nakane11/fix-text2wave-shebang
  [voicevox] Fix bin/text2wave shebang
* [voicevox] Fix bin/text2wave shebang
* Merge branch 'master' into power_switching_tools_ros
* Merge branch 'master' into fix-for-2404
* Merge pull request #567 from k-okada/iory/voicevox-docker
  add instrunction and to use voicevox docker container.
* Enhance VOICEVOX Docker support with flexible deployment options
  - Add docker_only option for launching Docker container without ROS sound_play node
  - Add remote connection support with automatic local/remote detection
  - Improve README with detailed usage examples for different deployment scenarios
  - Fix host binding issues to prevent port conflicts when connecting to remote containers
  - Support GPU acceleration with proper documentation
  Features:
  - docker_only: Launch only Docker container (useful for server deployment)
  - Automatic local/remote detection based on host parameter
  - Enhanced run-voicevox-docker script with remote connection handling
  - Comprehensive documentation for all usage patterns
* [voicevox] Update use_docker option
* [voicevox] Update use_docker option
* Update README
* Add execution for docker
* Merge pull request #554 from sawada10/voicevox-fix-bug-about-voicevox-server
  [voicevox] Do not launch voicevox server when docker is used
* Merge branch 'master' into power_switching_tools_ros
* Merge pull request #557 from k-okada/fix-voicevox-python-path-v2
  Fix voicevox python interpreter (duplicate of #552)
* set VENV_PATH in CMakeLists.txt and run @VEVN_PATH@/bin/python in run-voicevox.in
* [voicevox] Do not launch voicevox server when docker is used
* Merge pull request #546 from sawada10/voicevox-support-for-device-param
  [voicevox] add support for device param
* add support for device param
* Contributors: 73B2, Aoi Nakane, Iori Yanokura, Kei, Kei Okada, Michitoshi Tsubaki, Shun Hasegawa, Yoshiki Obinata, nakane11, sawada10

2.1.31 (2025-05-13)
-------------------
* add LICENSE files
* Contributors: Kei Okada

2.1.30 (2025-05-10)
-------------------
*  Add voicevox (AI speech synthesis) package (`#539 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/539>`_ , `#532 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/532>`_ , `#538 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/538>`_ )
* Contributors: Aoi Nakane, Iori Yanokura, Kei Okada, Naoto Tsukamoto, Shingo Kitagawa, Yoshiki Obinata

