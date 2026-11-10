^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros_speech_recognition
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* [ros_speech_recognition, respeaker_ros] remove flac binaries bundled with SpeechRecognition from the uv venv
  The 32bit flac-linux-x86 fails dh_shlibdeps of the buildfarm binary job.
  SpeechRecognition uses the flac in PATH first, which is an exec_depend.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [voicevox, chaplus_ros, emotion_analyzer, gdrive_ros, google_chat_ros, influxdb_store, power_switching_tools_ros, respeaker_ros, ros_speech_recognition, rospatlite, sesame_ros, switchbot_ros] depend on ros_environment for ROS2 buildfarm
  The buildfarm builds each package with only its own build dependencies,
  so without ros_environment $ROS_VERSION is empty and CMakeLists.txt takes
  the ROS1 branch (same as jsk_common#1835). voicevox also guesses
  ROS_VERSION from AMENT_PREFIX_PATH when it is unset.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* Merge pull request #580 from k-okada/ros2-google-cloud-stream
  [ros_speech_recognition] add GoogleCloudStream engine (ROS1/ROS2, rebase of #572)
* [ros_speech_recognition] use google-cloud-speech for GoogleCloud engine
  The GoogleCloud engine used google-api-python-client and oauth2client, which
  are not declared as dependencies. When they are installed in the venv,
  oauth2client imports the system pyOpenSSL, which does not work with the
  cryptography installed by google-auth (AttributeError: module 'lib' has no
  attribute 'GEN_EMAIL').
  Send the request with google-cloud-speech, already a dependency of the
  GoogleCloudStream engine. The config is still built as the REST JSON dict,
  so user_config (e.g. diarizationConfig) works as before, and show_all
  returns the same JSON dict. google-api-python-client is kept for
  python < 3.7, where google-cloud-speech is not available.
  Also send preferred_phrases as speechContexts (speechContext is not a field
  of RecognitionConfig), and remove a debug print.
* [ros_speech_recognition] report GoogleCloudStream stream errors instead of ignoring them
  Errors of the gRPC stream were swallowed by `except Exception: pass`, so an
  invalid credential or a disabled API looked like nothing was recognized.
  - errors that reconnecting cannot fix (Unauthenticated, PermissionDenied,
  InvalidArgument, NotFound) are logged once with logerr and stop the
  streaming loop, instead of reconnecting on every audio chunk
  - other errors (305s session limit, network) are logged with logwarn and the
  stream reconnects as before
  - streams closed by stop_stream() (e.g. self cancellation) are not logged
* [ros_speech_recognition] import numpy only for GoogleCloudStream engine
  python3 venv on kinetic/melodic can not see python-numpy (python2), and
  the engine is not available there anyway.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [ros_speech_recognition] add google-cloud-speech and numpy dependencies
  google-cloud-speech requires python >= 3.7, so skip it on kinetic/melodic
  python3 venv with a pip environment marker, and add it to pyproject.toml/uv.lock
  for ROS2.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [ros_speech_recognition] support GoogleCloudStream engine on both ROS1 and ROS2
  use ros_compat instead of rospy, import GoogleCloudStreamingRecognizer
  only when the engine is selected (google-cloud-speech is not available
  on python2), add GoogleCloudStream to cfg/ros2 and utf-8 coding header.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* add speech_recognition samples using GoogleCloudStream for README.md (#13)
* [ros_speech_recognition] log interim transcripts for GoogleCloudStream
* [ros_speech_recognition] add GoogleCloudStream engine for streaming recognition
  Wrap google-cloud-speech v1 StreamingRecognize so the node can emit interim
  transcripts on ~voice_interim_topic while still publishing the finalised
  utterance on ~voice_topic.
* Merge branch 'master' into fix_hoya
* Merge pull request #576 from k-okada/ros2-speech-recognition-only
  [ros_speech_recognition] add ROS2 support with jsk_ros1_ros2_compat
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
* [ros_speech_recognition] README: ROS2
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [ros_speech_recognition] add ROS2 launch files
  speech_recognition.launch.xml and parrotry.launch.xml, with the args of
  the ROS1 launch files. julius_ros and aques_talk (use_google:=false and
  ja-JP of parrotry.launch) have no ROS2 port, so parrotry.launch.xml
  takes the engine and speaks with sound_play. From Michi-Tsubaki's ROS2
  port.
  Co-authored-by: Michitoshi Tsubaki <tsubaki@jsk.t.u-tokyo.ac.jp>
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [ros_speech_recognition] build on ROS2
  - CMakeLists.txt: the ROS2 build is prepended and the ROS1 original is
  wrapped in else()/endif() unchanged (41 lines added, 0 deleted):
  - the package and cfg/ros2/SpeechRecognitionConfig.py (as
  ros_speech_recognition.cfg.SpeechRecognitionConfig) are installed,
  - SpeechRecognition/vosk are not available from apt, and
  catkin_virtualenv has no ROS2 port, so speech_recognition_node.py
  runs in a uv venv made from pyproject.toml/uv.lock (the same
  versions as requirements.txt) with
  jsk_ros1_ros2_compat_generate_uv_venv(), installed to share/ (so it
  is shipped in the deb); the scripts are installed with
  jsk_ros1_ros2_compat_install_python() (a python3 or the venv's
  shebang),
  - the vosk models are downloaded with install_trained_data.py
  (jsk_data), as the ROS1 install_trained_data target does, at install
  time into share/ros_speech_recognition/trained_data.
  - package.xml: format 3, with $ROS_VERSION conditions on the ROS1/ROS2
  dependencies and build_type; jsk_ros1_ros2_compat (>= 3.0.2) is a ROS2
  dependency, as ROS1 falls back to rospy without it.
  - AMENT_IGNORE is removed.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [ros_speech_recognition] install_trained_data.py: add --data-dir
  The directory to download the vosk models to: relative to the package
  (default trained_data, as before), or absolute, for ROS2, where the
  models are downloaded at install time into the share directory, which
  ament_index does not find while the package is being built (see
  CMakeLists.txt).
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [ros_speech_recognition] recognize_vosk.py: find the vosk models with ament_index on ROS2
  trained_data/ is in the installed share directory on ROS2 (see
  install_trained_data.py), found with ament_index instead of rospkg.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [ros_speech_recognition] speech_recognition_candidates_to_string.py: run on ROS2 without ConnectionBasedTransport
  jsk_topic_tools has no ROS2 python ConnectionBasedTransport, so on ROS2
  __main_\_ always subscribes and publishes the first transcript; the
  class is the ROS1 one.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [ros_speech_recognition] parrot_node.py: give sound_play's SoundClient an own node on ROS2
  ROS2's sound_play.libsoundplay.SoundClient(node, blocking=True) spins
  its node by itself with rclpy.spin_until_future_complete() (see
  sound_play's say.py), so the node of a subscription callback can not be
  passed to it: on ROS2, __main_\_ creates another node for it, which is
  not spun by ros_compat.spin(). The SoundClient of ROS1 is unchanged.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [ros_speech_recognition] spin with multiple threads, start the continuous mode after spinning
  The speech_recognition service callback waits for the audio topic and
  the sound_play action, which works with rospy, as it runs each callback
  in its own thread, but not with rclpy's default single-threaded
  executor: ros_compat.init_node(..., multi_threaded=True) (ignored on
  ROS1). For the same reason, the continuous mode, which waits for the
  audio topic too, starts in a thread after spinning starts. From
  Michi-Tsubaki's ROS2 port (MultiThreadedExecutor with reentrant
  callbacks, and a timer to start).
  Co-authored-by: Michitoshi Tsubaki <tsubaki@jsk.t.u-tokyo.ac.jp>
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [ros_speech_recognition] self cancellation with the status topics of the tts actions
  Whether the robot is speaking came from actionlib's
  action_client.last_status_msg, the status of all the goals of the
  action, which ROS2 action clients do not keep. Subscribe to the status
  topic of each tts action instead (<action>/status of actionlib_msgs on
  ROS1, <action>/_action/status of action_msgs on ROS2), as
  Michi-Tsubaki's ROS2 port does.
  Co-authored-by: Michitoshi Tsubaki <tsubaki@jsk.t.u-tokyo.ac.jp>
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [ros_speech_recognition] audio subscription on ROS2
  The audio topic is subscribed with a depth of 10 on ROS2 (rclpy needs
  one, and ros_compat's default 1 may drop audio; ROS1 keeps rospy's
  unbounded queue), and unsubscribed with destroy_subscription().
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [ros_speech_recognition] fix the speech_recognition service with the Google engine
  recognize_google() is called with show_all=True, so it returns a dict
  of all the alternatives (or [] for no result), and the service failed
  on result.encode(). Take the best alternative and its confidence as
  audio_cb() of the continuous mode does. From Michi-Tsubaki's ROS2 port.
  Co-authored-by: Michitoshi Tsubaki <tsubaki@jsk.t.u-tokyo.ac.jp>
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [ros_speech_recognition] disable the recognizer's dynamic energy threshold with the config
  dynamic_energy_threshold:=false kept speech_recognition.Recognizer's own
  dynamic_energy_threshold (default True), so energy_threshold was still
  adjusted in listen(). From Michi-Tsubaki's ROS2 port.
  Co-authored-by: Michitoshi Tsubaki <tsubaki@jsk.t.u-tokyo.ac.jp>
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [ros_speech_recognition] add ros_speech_recognition.compat for jsk_common < 3.0
  jsk_ros1_ros2_compat is only released with jsk_common >= 3.0, which is
  not available for every ROS1 distro, so this package depends on it for
  ROS2 only, and src/ros_speech_recognition/compat.py gives `ros_compat`
  and `ROS_VERSION`: jsk_ros1_ros2_compat's rospy_rclpy_compat, or the same
  functions forwarded to rospy when it can not be imported (ROS1 only).
  The imports were switched by
  cd ros_speech_recognition
  sed -i 's/^from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat$/from ros_speech_recognition.compat import ros_compat/; s/^from jsk_ros1_ros2_compat.rospy_rclpy_compat import ROS_VERSION$/from ros_speech_recognition.compat import ROS_VERSION/' \
  $(grep -rl '^from jsk_ros1_ros2_compat' scripts src --include=*.py)
  Importing ros_speech_recognition.compat runs the package's __init_\_.py,
  which imports client.py (SpeechRecognitionClient, with rospy and
  dynamic_reconfigure), so __init_\_.py imports it only on ROS1, where it
  is used; SpeechRecognitionClient stays ROS1 only.
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
* [ros_speech_recognition] rospy/actionlib/dynamic_reconfigure -> jsk_ros1_ros2_compat
  Use jsk_ros1_ros2_compat's rospy_rclpy_compat as `ros_compat` instead of
  rospy, actionlib and dynamic_reconfigure, so that the nodes run on ROS1
  and ROS2. `node` is None on ROS1.
  1. Mechanical part, generated without manual edits by
  cd ros_speech_recognition
  FILES=$(grep -l '^import rospy$' scripts/*.py src/ros_speech_recognition/*.py)
  sed -i '/^import rospy$/a from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat' $FILES
  python3 rospy_to_ros_compat.py $FILES
  rospy_to_ros_compat.py (below) converts the rospy calls whose
  ros_compat functions take the same arguments (get_param, has_param,
  log*, is_shutdown, sleep, on_shutdown, spin, Subscriber/Publisher ->
  create_subscription/create_publisher, init_node), adding the node
  argument:
  - `node` in __main_\_ (from ros_compat.init_node()) and in __init_\_ of
  the classes which need a node, which get a `node=None` argument,
  - `self.node` in their other methods (set in __init_\_),
  - None in module-level functions and classes which only log,
  and removes `import rospy` when no rospy.xxx is left.
  2. By hand, in speech_recognition_node.py:
  - the node is passed to ROSAudio's audio stream,
  - rospy.Service -> ros_compat.create_service: the callbacks take
  (request, response) and return the response, as rclpy's,
  - rospy.Time/Duration/Timer -> float seconds of ros_compat.now_sec()
  (tts_tolerance too) and ros_compat.create_timer(),
  - actionlib.SimpleActionClient -> ros_compat.create_simple_action_client
  (timeouts in seconds); send_goal_and_wait() returns whether the goal
  finished, not its state, and ROS2's client has no get_state(), so
  the "aborted" warning of the sound signals uses get_state() when
  available,
  - dynamic_reconfigure -> ros_compat.create_reconfigure_server: its
  parameters are ROS2 parameters with the defaults of
  cfg/ros2/SpeechRecognitionConfig.py, which is installed as
  ros_speech_recognition.cfg.SpeechRecognitionConfig on ROS2 (the
  defaults of cfg/SpeechRecognition.cfg and the engine constants), and
  the same config_callback(config, level) is called on
  `ros2 param set`; ROS1 keeps dynamic_reconfigure,
  - a str is logged instead of an exception (rclpy's loggers take a
  str),
  - the message types: sound_play's messages and action are in
  sound_play_msgs and GoalStatus in action_msgs on ROS2,
  - `import actionlib`, `import rospy` and the dynamic_reconfigure
  import are removed.
  rospy_to_ros_compat.py:
  #!/usr/bin/env python3
  """rospy.xxx(...) -> ros_compat.xxx(<node>, ...) for the functions with the
  same arguments in jsk_ros1_ros2_compat (Subscriber/Publisher ->
  create_subscription/create_publisher). <node> is:
  - `node` in __main_\_ (`rospy.init_node(...)` -> `node = ros_compat.init_node(...)`,
  `[x = ]Foo()` -> `[x = ]Foo(node)`, `node = Foo()` -> `Foo(node)`) and in
  `__init_\_` of classes which need a node (`node=None` is added),
  - `self.node` in their other methods (`self.node = node` is added to __init_\_),
  - None in module-level functions and in classes which only log.
  `import rospy` is removed when no rospy.xxx is left.
  usage: rospy_to_ros_compat.py FILE..."""
  import re
  import sys
  LOG = ['loginfo', 'logwarn', 'logerr', 'logdebug']
  NODE = ['get_param', 'has_param', 'is_shutdown', 'sleep', 'on_shutdown',
  'spin', 'Subscriber', 'Publisher']
  RENAME = {'Subscriber': 'create_subscription', 'Publisher': 'create_publisher'}
  CALL = re.compile(r'rospy\.(%s)\((\)|\s*$)?' % '|'.join(LOG + NODE))
  def scopes(lines):
  """[(class name or None, def name or None, in __main_\_)] for each line"""
  stack, result = [], []
  for line in lines:
  indent = len(line) - len(line.lstrip())
  if line.strip() and not line.lstrip().startswith(('#', ')')):
  while stack and stack[-1][0] >= indent:
  stack.pop()
  m = re.match(r'\s*(class|def)\s+(\w+)', line)
  if m:
  stack.append((indent, m.group(1), m.group(2)))
  elif re.match(r'if __name_\_ == .__main_\_.:', line):
  stack.append((indent, 'main', None))
  # the innermost method (a def directly in a class)
  cls, func = None, None
  for outer, inner in zip(stack, stack[1:]):
  if outer[1] == 'class' and inner[1] == 'def':
  cls, func = outer[2], inner[2]
  main = any(s[1] == 'main' for s in stack)
  result.append((cls, func, main))
  return result
  def convert(text):
  lines = text.split('\n')
  sc = scopes(lines)
  # classes needing a node, and those using it out of __init\_\_
  need, self_node = set(), set()
  for line, (cls, func, main) in zip(lines, sc):
  for m in CALL.finditer(line):
  if cls and m.group(1) in NODE:
  need.add(cls)
  if func != '__init_\_':
  self_node.add(cls)
  out = []
  in_init_sig = None
  for line, (cls, func, main) in zip(lines, sc):
  if main:
  node = 'node'
  elif cls in need:
  node = 'node' if func == '__init_\_' else 'self.node'
  else:
  node = 'None'
  def repl(m):
  name = RENAME.get(m.group(1), m.group(1))
  if m.group(2) == ')':
  return 'ros_compat.%s(%s)' % (name, node)
  if m.group(2) is not None:  # arguments from the next line
  return 'ros_compat.%s(%s,' % (name, node) + m.group(2)
  return 'ros_compat.%s(%s, ' % (name, node)
  if main and re.match(r'\s*rospy\.init_node\(', line):
  line = line.replace('rospy.init_node(', 'node = ros_compat.init_node(')
  elif main:
  line = CALL.sub(repl, line)
  for c in need:
  line = re.sub(r'\bnode = %s\(\)' % c, '%s(node)' % c, line)
  line = re.sub(r'\b%s\(\)' % c, '%s(node)' % c, line)
  else:
  line = CALL.sub(repl, line)
  # def __init_\_(self, ...): -> def __init_\_(self, ..., node=None):
  if cls in need and re.match(r'\s*def __init_\_\(', line):
  in_init_sig = (cls, len(line) - len(line.lstrip()))
  if in_init_sig and line.rstrip().endswith('):'):
  line = re.sub(r'\(self\):$', '(self, node=None):', line.rstrip())
  if not line.endswith('node=None):'):
  line = line[:-2] + ', node=None):'
  out.append(line)
  if in_init_sig[0] in self_node:
  out.append(' ' * (in_init_sig[1] + 4) + 'self.node = node')
  in_init_sig = None
  continue
  out.append(line)
  text = '\n'.join(out)
  if 'rospy.' not in text:
  text = re.sub(r'(?m)^import rospy\n', '', text)
  return text
  for path in sys.argv[1:]:
  with open(path) as f:
  text = f.read()
  with open(path, 'w') as f:
  f.write(convert(text))
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
* Merge pull request #564 from Jun-young2eyo/fix-for-2404
  [ros_speech_recognition] Make speech_recognition.launch work in Ubuntu 24.04
* Merge pull request #4 from pazeshun/fix-for-2404-refined-oga
  [ros_speech_recognition] To support the case when speech_recognition and sound_play run on different PCs, keep sound file path when it is provided by user, keep node running even when sound file is not found, and warn when sound_play is aborted
* [ros_speech_recognition] To support the case when speech_recognition and sound_play run on different PCs, keep sound file path when it is provided by user, keep node running even when sound file is not found, and warn when sound_play is aborted
* Merge pull request #2 from pazeshun/fix-for-2404-compatible
  [ros_speech_recognition] Call tobytes only when tobytes exists
* [ros_speech_recognition] Call tobytes only when tobytes exists
  6e40fa47b393b04f62ff82c693bc1cb0a71a039c is wrong.
  In fact, tobytes() does not exist in Python 2.7
* Merge pull request #1 from pazeshun/fix-for-2404-readme
  [ros_speech_recognition] Add info about .oga to README
* [ros_speech_recognition] Add info about .oga to README
* [ros_speech_recognition] Fall back to .oga sound signal files
* [ros_speech_recognition] Use tobytes instead of tostring for Python >= 3.9
  tostring() is removed in Python 3.9.
  tobytes() works in the same way as tostring(), so we can use tobytes().
  tobytes() exists in Python >= 2.7, so we do not break backward compatiblity when we replace tostring() with tobytes()
* Contributors: Choi Junyoung, Iori Yanokura, Junyoung Choi, Kei Okada, Shun Hasegawa, Tomoka Sawada

2.1.31 (2025-05-13)
-------------------

2.1.30 (2025-05-10)
-------------------

2.1.29 (2025-01-05)
-------------------
* [doc] fix typo in jsk_3rdparty/ros_speech_recognition/README.md (`#499 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/499>`_)
* Contributors: Yukina Iwata

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
* [ros_speech_recognition] Add vosk engine (`#474 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/474>`_)
* Pr/use sound themes freedesktop (`#472 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/472>`_)
* add test to check if ros node is loadable (`#463 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/463>`_)
* add self.conf_thresh in __init_\_ function (`#457 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/457>`_)
* [ros_speech_recognition] add ubuntu-sounds dependency (`#453 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/453>`_)
* [ros_speech_recognition] Return if result is empty (`#443 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/443>`_)
* [ros_speece_recognition] Set confidence value of google (`#434 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/434>`_)
* [ros_speech_recognition] add parrotry.launch (`#414 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/414>`_)
* [ros\_ speech_recognition] update default arg for speech_recognition.launch (`#412 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/412>`_)
* [ros_speech_recogniton, respeaker_ros] add confidence field (`#411 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/411>`_)
* [ros_speech_recognition] add self cancellation for speech recogntion (`#413 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/413>`_)
* [`#405 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/405>`_ and `#410 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/410>`_] Fix CI (`#415 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/415>`_)
* add ROS interface for https://cloud.google.com/natural-language (`#304 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/304>`_)
* GithubAction: add test for  aarch64(melodic) / indigo (arm64) (`#365 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/365>`_)

  * pgm_learner/respeaker_ros/ros_speech_recognition/rosping: increase time-limit/wait-time

* Explicit python interpreter in catkin_virtualenv (`#367 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/367>`_)
* .github/workflow:  integrate all yaml to one (`#338 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/338>`_)
* [ros_speech_recognition] Fixed the behavior of launch file (`#336 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/336>`_)
* [ros_speech_recognition] add auto_start in speech_recognition_node.py (`#301 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/301>`_)
* [ros_speech_recognition] add SpeechRecognitionCandidatesToString node (`#303 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/303>`_)
* Enable sound play flag (`#315 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/315>`_)
* Contributors: Aiko Ichikura, Aoi Nakane, Kei Okada, Koki Shinjo, Naoto Tsukamoto, Naoya Yamaguchi, Shingo Kitagawa, Yoshiki Obinata, Iory Yanokura

2.1.24 (2021-07-26)
-------------------

2.1.23 (2021-07-21)
-------------------

2.1.22 (2021-06-10)
-------------------
* enable to change topic name from speech_recognition.launch (`#254 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/254>`_)
* support SpeakerDiarization, see https://cloud.google.com/speech-to-text/docs/reference/rest/v1/speech/recognize#SpeechRecognitionAlternative (`#244 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/244>`_)

  * [ros_speech_recognition] Add doc to speech_recognition.launch
    add doc to args, and we need to use rosparm for device, not param. because 'device: ' causes
    load_parameters: unable to set parameters (last param was [/speech_recognition/depth=16]): cannot marshal None unless allow_none is enabled  error
  * more exception message for self.recognize

* Use PYTHON_INTERPRETER python3 in ros_speech_recognition (`#225 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/225>`_)

* Contributors: Kei Okada, Naoya Yamaguchi, Shingo Kitagawa

2.1.21 (2020-08-19)
-------------------
* add missing packages, closes https://github.com/ros/rosdistro/pull/26216 (`#211 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/211>`_)

* Contributors: Kei Okada

2.1.20 (2020-08-07)
-------------------

2.1.19 (2020-07-21)
-------------------
* Fixed issue `#201 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/201>`_ as requested, see https://github.com/jsk-ros-pkg/jsk_3rdparty/pull/202
* Contributors: MrMarshy

2.1.18 (2020-07-20)
-------------------
* Fix for noetic (`#200 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/200>`_)

  * fix 2to3, with print, raise, exception

* [ros_speech_recognition] Enable multi channel audio recognition (`#198 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/198>`_)

  * adjust type code to the CPU platform
  * replace rosparam name: channels -> n_channel
  * add rosparam description to README
  * enable multi channel audio recognition

* Add args to ros_speech_recognition (`#197 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/197>`_)

  * Add flac as run_depend for SpeechRecognition pip package
  * Use catkin_virtualenv to use SpeechRecognition pip package
  * Add arguments and params to pass rostest
  * Add test for ros_speech_recognition
  * add args to launch
  * add pip install to tutorials
  * add param description to README

* Contributors: Kei Okada, Naoya Yamaguchi

2.1.17 (2020-04-16)
-------------------

2.1.16 (2020-04-16)
-------------------

2.1.15 (2019-12-12)
-------------------

2.1.14 (2019-11-21)
-------------------
* set SoundRequest.volume for kinetic (`#173 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/173>`_)
* Contributors: Kei Okada

2.1.13 (2019-07-10)
-------------------

2.1.12 (2019-05-25)
-------------------
* fixes GoogleCloud auth (`#158 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/158>`_)
* Contributors: jonasius

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
* ros_speech_recognition: add continuous mode (`#127 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/127>`_)
* ros_speech_recognition: add README (`#123 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/123>`_)
* add ros_speech_recognition package (`#121 <https://github.com/jsk-ros-pkg/jsk_3rdparty/issues/121>`_)
* Contributors: Yuki Furuta

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
