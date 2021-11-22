voice_text
==========

ROS Interface for HOYA VoiceText Speech Synthesis Engine

## Installation

### 1. Install VoiceText SDK
#### If you have voicetext sdk install binary, please follow the official guide and install both engine and SDK
#### If you don't have the sdk install binary but have voice text API binary, please follow the guide below.
1. Install VoiceText Engine by official guide
2. Copy VoiceText API binaries to VoiceText binary directory
  VoiceText API package includes binary libraries and header file. You have to copy those of them to specific directory by executing following commands.
  ```bash
  cd /path_to_api_package_directory # e.g. cd ~/Downloads/RS_VTAPI_SDK_Linux_4.3.0.2/20201113_VTAPI4.3.0.2_LINUX
  cd bin/x64 # You have to cd x86 if your system is x86 architecture
  # Assuming VoiceText engine's talker is hikari, type is D16. If it is different, please set appropriate directory.
  sudo cp -a * /usr/vt/hikari/D16/bin # Don't forget to add -a not to break symbolic link. 
  cd ../../include/
  sudo mkdir /usr/vt/hikari/D16/inc # not include, but inc
  sudo cp vtapi.h /usr/vt/hikari/D16/inc
  ```
### 2. Put license file
### 3. Build this package

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
