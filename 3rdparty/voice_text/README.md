voice_text
==========

ROS Interface for HOYA VoiceText Speech Synthesis Engine

## Installation

1. Install VoiceText SDK
2. Put license file
3. Build this package

```bash
cd /path/to/catkin_workspace
catkin build voice_text
```

## Usage

1. Launch `voice_text` node

```bash
roslaunch voice_text voice_text.launch
```

**TIPS** If you want to launch VoiceText engine on remote machine, try creating launch file like below:

```xml
<launch>
  <machine name="remote" address="192.168.0.3" />
  <include file="$(find voice_text)/launch/voice_text.launch">
    <arg name="machine" value="remote" />
  </include>
</launch>
```

2. Say

Now speech synthesis is enabled.

```bash
rosrun sound_play say.py Hello! dummy 1.0 robotsound:=robotsound_jp
```

Robot says "Hello!"

## Reference

### Service

* `text_to_speech` (`voice_text/TextToSpeech`)

  Set `text_path` for path to text file that contains speech sentences.
  Set `wave_path` for path to generated wave file to be generated.
  If `ok` in response is set to `true`, wave file is generated successfully.

### Parameters

* `~db_path` (String, default: path found at build time (e.g., `"/var/vt/sayaka/M16"`))

  Path to VoiceText database directory.

* `~license_path` (String, default: `""`)

  Path to VoiceText license file.
  If this parameter is empty, SDK tries to search `data-common/verify` directory relative to `db_path`.

### Dynamic Parameters

* `~pitch` (Int, default: `100`)

  Pitch of voice by percentage. Speaks higher if this parameter is higher than `100`.

* `~speed` (Int, default: `100`)

  Speed of speech by percentage.

* `~volume` (Int, default: `100`)

  Volumet of speech by percentage.

* `~pause` (Int, default: `800`)

  Pause duration between sentences by milliseconds.
