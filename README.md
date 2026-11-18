# jsk_3rdparty

[![ROS 1](https://github.com/jsk-ros-pkg/jsk_3rdparty/actions/workflows/config.yml/badge.svg?branch=master)](https://github.com/jsk-ros-pkg/jsk_3rdparty/actions/workflows/config.yml)
[![ROS 2](https://github.com/jsk-ros-pkg/jsk_3rdparty/actions/workflows/ros2.yaml/badge.svg?branch=master)](https://github.com/jsk-ros-pkg/jsk_3rdparty/actions/workflows/ros2.yaml)
[![License](https://img.shields.io/badge/license-BSD--3--Clause-blue)](jsk_3rdparty/package.xml)

[![jazzy](https://img.shields.io/ros/v/jazzy/jsk_3rdparty)](https://index.ros.org/r/jsk_3rdparty/#jazzy)
[![kilted](https://img.shields.io/ros/v/kilted/jsk_3rdparty)](https://index.ros.org/r/jsk_3rdparty/#kilted)
[![lyrical](https://img.shields.io/ros/v/lyrical/jsk_3rdparty)](https://index.ros.org/r/jsk_3rdparty/#lyrical)
[![noetic](https://img.shields.io/ros/v/noetic/jsk_3rdparty)](https://index.ros.org/r/jsk_3rdparty/#noetic)
[![ROS One](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/jsk-ros-pkg/jsk_3rdparty/badges/ros_one.json)](https://ros.packages.techfak.net/)

ROS wrappers of third-party libraries and web services used in the JSK Lab (the University of Tokyo): speech
synthesis and recognition, chat and cloud services, smart-home devices, PDDL planners, and libraries packaged for
ROS. The packages marked ROS 2 below build from the same source for ROS 1 (catkin) and ROS 2 (ament); they switch by
`$ROS_VERSION`, and `jsk_ros1_ros2_compat` (of [jsk_common](https://github.com/jsk-ros-pkg/jsk_common)) keeps the
code shared. The other packages are of ROS 1 only and carry an `AMENT_IGNORE`.

## Packages

| Package | ROS 1 | ROS 2 | What it is |
|---|:---:|:---:|---|
| [chaplus_ros](chaplus_ros) | ✓ | ✓ | Chat with the chaplus, A3RT Talk or mebo dialogue services |
| [emotion_analyzer](emotion_analyzer) | ✓ | ✓ | Emotions of a text or a voice by the Hume AI API |
| [gdrive_ros](gdrive_ros) | ✓ | ✓ | Uploading files to Google Drive |
| [google_chat_ros](google_chat_ros) | ✓ | ✓ | Sending and receiving messages of Google Chat |
| [influxdb_store](influxdb_store) | ✓ | ✓ | Storing topics (battery, joint states, network, robot pose) into InfluxDB, to be shown on Grafana |
| [jsk_power_switching_tools](jsk_power_switching_tools) | ✓ | ✓ | Power cycling USB devices: USB-serial troubleshooter, USB PPPS hubs |
| [nfc_ros](nfc_ros) | ✓ | ✓ | Reading NFC tags |
| [respeaker_ros](respeaker_ros) | ✓ | ✓ | Driver of the ReSpeaker Mic Array v2.0: audio, sound direction, voice activity |
| [ros_speech_recognition](ros_speech_recognition) | ✓ | ✓ | Speech recognition by the Python SpeechRecognition library (Google, Google Cloud, Vosk, Sphinx, ...) |
| [rospatlite](rospatlite) | ✓ | ✓ | Driver of the PATLITE signal tower (network) |
| [rosping](rosping) | ✓ | ✓ | ICMP ping to hosts, as a topic |
| [sesame_ros](sesame_ros) | ✓ | ✓ | Locking and unlocking the Sesame smart lock |
| [switchbot_ros](switchbot_ros) | ✓ | ✓ | Controlling SwitchBot devices by the SwitchBot API |
| [voicevox](3rdparty/voicevox) | ✓ | ✓ | VOICEVOX: Japanese text-to-speech, for `sound_play` |
| [zdepth](3rdparty/zdepth), [zdepth_image_transport](zdepth_image_transport) | ✓ | ✓ | Zdepth depth image compression, as an image_transport plugin |
| [aques_talk](3rdparty/aques_talk) | ✓ | | AquesTalk2: Japanese text-to-speech |
| [google_cloud_texttospeech](3rdparty/google_cloud_texttospeech) | ✓ | | Google Cloud Text-to-Speech for `sound_play` |
| [voice_text](3rdparty/voice_text) | ✓ | | HOYA VoiceText (ReadSpeaker) text-to-speech |
| [julius](3rdparty/julius), [julius_ros](julius_ros) | ✓ | | Julius speech recognition engine and its ROS interface |
| [webrtcvad_ros](webrtcvad_ros) | ✓ | | Voice activity detection of an audio topic by webrtcvad |
| [dialogflow_task_executive](dialogflow_task_executive) | ✓ | | Dialogue by Google Dialogflow, launching apps by it |
| [ros_google_cloud_language](ros_google_cloud_language) | ✓ | | Google Cloud Natural Language API |
| [rostwitter](rostwitter) | ✓ | | Tweeting from ROS |
| [opt_camera](opt_camera) | ✓ | | Driver of the OPT NM33 omnidirectional camera |
| [downward](3rdparty/downward), [ff](3rdparty/ff), [ffha](3rdparty/ffha), [lpg_planner](3rdparty/lpg_planner) | ✓ | | PDDL planners |
| [bayesian_belief_networks](bayesian_belief_networks), [pgm_learner](pgm_learner) | ✓ | | Bayesian belief networks, and their parameter estimation |
| [assimp_devel](3rdparty/assimp_devel), [libcmt](3rdparty/libcmt), [libsiftfast](3rdparty/libsiftfast), [nlopt](3rdparty/nlopt), [osqp](3rdparty/osqp), [slic](3rdparty/slic) | ✓ | | Libraries packaged for ROS: assimp, CMT tracker, SIFT, NLopt, OSQP, SLIC superpixels |
| [mini_maxwell](3rdparty/mini_maxwell) | ✓ | | Setting up the Mini Maxwell network emulator |
| [collada_urdf_jsk_patch](jsk_ros_patch/collada_urdf_jsk_patch), [laser_filters_jsk_patch](jsk_ros_patch/laser_filters_jsk_patch) | ✓ | | Patched versions of collada_urdf and laser_filters |
| [hark_jsk_plugins](hark_jsk_plugins) | | | Plugins of HARK (not a ROS package) |
| [jsk_3rdparty](jsk_3rdparty) | ✓ | | The metapackage |

The README of each package (linked above) lists its nodes, launch files, topics and parameters. Many of these
packages are clients of web services or drivers of devices, so they need an API key, a credential file or the
hardware: see the "Requirements" of each README.

## Install

```bash
# ROS 1 noetic
sudo apt install ros-noetic-jsk-3rdparty
# ROS One (https://ros.packages.techfak.net)
sudo apt install ros-one-jsk-3rdparty
# ROS 2 (jazzy, kilted, lyrical, rolling): once released (see the badges above), per package, e.g.
sudo apt install ros-$ROS_DISTRO-voicevox
```

From source:

```bash
mkdir -p ~/ws/src && cd ~/ws/src
git clone https://github.com/jsk-ros-pkg/jsk_3rdparty.git
cd ~/ws
# ROS 2: the packages of ROS 1 only are skipped by their AMENT_IGNORE
rosdep install -y -r --from-paths src --ignore-src
colcon build --symlink-install
# ROS 1: rosdep also skips the packages with AMENT_IGNORE, so remove the markers in a ROS 1 workspace
find src/jsk_3rdparty -name AMENT_IGNORE -delete
rosdep install -y -r --from-paths src --ignore-src
catkin build
```

Note: in ROS 1 this repository contains non-catkin packages (pure-cmake directories such as `osqp`, `libcmt`,
`slic` and `zdepth`), so use either `catkin build` or `catkin_make_isolated`. `catkin_make` will not work.
