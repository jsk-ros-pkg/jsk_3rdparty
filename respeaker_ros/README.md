respeaker_ros
=============

A ROS package for the ReSpeaker Mic Array v2.0. `respeaker_node.py` publishes the audio of the microphone array,
voice activity (VAD), the direction of arrival (DoA) of the sound and the audio of each utterance, controls the LEDs,
and exposes the parameters of the DSP of the device. `speech_to_text.py` sends each utterance to the Google Web Speech
API (through [SpeechRecognition](https://pypi.org/project/SpeechRecognition/)) and publishes the text.
The same scripts run on ROS 1 and ROS 2.

## Supported Devices

- [ReSpeaker Mic Array v2.0](http://wiki.seeedstudio.com/ReSpeaker_Mic_Array_v2.0/) (USB `2886:0018`)

    ![Respeaker Mic Array v2.0](https://github.com/SeeedDocument/ReSpeaker_Mic_Array_V2/raw/master/img/Hardware%20Overview.png)

## Requirements

- The device, with the 6 channels firmware (see [Preparation](#preparation)).
- `speech_to_text.py` needs internet access to the Google Web Speech API (no key is needed).
- SpeechRecognition, pyusb and pixel-ring are installed into a virtualenv of the package at build time
  (ROS 1: catkin_virtualenv with `requirements.txt`; ROS 2: [uv](https://docs.astral.sh/uv/) with
  `pyproject.toml`/`uv.lock`, run `uv lock` after changing `pyproject.toml`).

## Preparation

1. Install this package

    ```bash
    sudo apt install ros-$ROS_DISTRO-respeaker-ros
    ```

    or build it from source (`catkin build respeaker_ros` on ROS 1, `colcon build --packages-up-to respeaker_ros`
    on ROS 2).

1. Register the udev rule `config/60-respeaker.rules` (`MODE:="0666"` for the USB device `2886:0018`), so that
   the device can be accessed without root: copy it, reload the rules and apply them to the connected device (or
   re-connect the device)

    ```bash
    # ROS 1
    sudo cp -f $(rospack find respeaker_ros)/config/60-respeaker.rules /etc/udev/rules.d/60-respeaker.rules
    # ROS 2
    sudo cp -f $(ros2 pkg prefix respeaker_ros)/share/respeaker_ros/config/60-respeaker.rules /etc/udev/rules.d/60-respeaker.rules
    # both
    sudo udevadm control --reload-rules && sudo udevadm trigger
    ```

1. Update the firmware to the 6 channels version

    ```bash
    git clone https://github.com/respeaker/usb_4_mic_array.git
    cd usb_4_mic_array
    sudo python dfu.py --download 6_channels_firmware.bin
    ```

## Usage

### ROS 2

```bash
ros2 launch respeaker_ros sample_respeaker.launch.xml
# for Japanese
ros2 launch respeaker_ros sample_respeaker.launch.xml language:=ja-JP
ros2 topic echo /sound_direction       # DoA [deg]
ros2 topic echo /sound_localization    # DoA as a pose
ros2 topic echo /is_speeching          # VAD
ros2 topic echo /speech_to_text        # recognized text
# LED color (back to the DoA display after 3 seconds)
ros2 topic pub --once /status_led std_msgs/msg/ColorRGBA "{r: 0.0, g: 0.0, b: 1.0, a: 0.3}"
# a parameter of the device (instead of dynamic_reconfigure)
ros2 param set /respeaker_node AGCGAIN 5.5
```

### ROS 1

```bash
roslaunch respeaker_ros sample_respeaker.launch
rostopic echo /sound_direction
rostopic echo /speech_to_text
# Japanese results
rostopic echo --filter "print(m.transcript[0])" /speech_to_text
rostopic pub --once /status_led std_msgs/ColorRGBA "{r: 0.0, g: 0.0, b: 1.0, a: 0.3}"
rosrun rqt_reconfigure rqt_reconfigure   # parameters of the device
```

### Launch arguments (`sample_respeaker.launch` / `sample_respeaker.launch.xml`)

| Argument | Default | Meaning |
|---|---|---|
| `publish_tf` | `true` | Publish a static TF `map` -> `respeaker_base` (ROS 1: tf, ROS 2: tf2_ros) |
| `launch_respeaker` | `true` | Launch `respeaker_node` |
| `launch_soundplay` | `true` | Launch sound_play (its action `sound_play` is used for self cancellation) |
| `audio` | `speech_audio` | Input audio topic of `speech_to_text` |
| `speech_to_text` | `speech_to_text` | Output topic of `speech_to_text` |
| `language` | `en-US` | Language of the recognition |
| `self_cancellation` | `true` | Ignore the audio while sound_play is speaking |

`speech_to_text` is launched with `tts_tolerance: 0.5`.

## Interface

### respeaker_node.py

Published topics:

- `audio` (`audio_common_msgs/AudioData`): raw audio of channel 0 (the processed channel for ASR), 16 kHz, 16 bit, mono
- `speech_audio` (`audio_common_msgs/AudioData`): the audio of each utterance detected by VAD
- `is_speeching` (`std_msgs/Bool`, latched): VAD, on change
- `sound_direction` (`std_msgs/Int32`, latched): DoA in degrees, on change
- `sound_localization` (`geometry_msgs/PoseStamped`, latched): DoA as a pose in `~sensor_frame_id`

Subscribed topics:

- `status_led` (`std_msgs/ColorRGBA`): set the color of the LEDs for 3 seconds

Parameters:

| Name | Type | Default | Meaning |
|---|---|---|---|
| `~update_rate` | double | `10.0` | Rate [Hz] of reading VAD and DoA |
| `~sensor_frame_id` | string | `respeaker_base` | Frame of `sound_localization` |
| `~doa_xy_offset` | double | `0.0` | Distance [m] of the pose of `sound_localization` from the frame origin |
| `~doa_yaw_offset` | double | `90.0` | Offset [deg] of the DoA |
| `~speech_prefetch` | double | `0.5` | Seconds of audio before the start of an utterance added to `speech_audio` |
| `~speech_continuation` | double | `0.5` | Seconds of silence which end an utterance |
| `~speech_max_duration` | double | `7.0` | Utterances longer than this [s] are not published |
| `~speech_min_duration` | double | `0.1` | Utterances shorter than this [s] are not published |
| `~sample_duration` | double | none | Seconds of audio per `audio` message (none: 1024 frames, 64 ms) |
| `~suppress_pyaudio_error` | bool | `true` | Hide the error messages of ALSA |

The DSP parameters of the device (e.g. `AGCONOFF`, `AGCGAIN`, `STATNOISEONOFF`, `GAMMAVAD_SR`, the writable entries of
`PARAMETERS` in `src/respeaker_ros/__init__.py`) are read from the device at start and written to it when changed:
with dynamic_reconfigure (`cfg/Respeaker.cfg`) on ROS 1, and as ROS 2 parameters of `respeaker_node` on ROS 2
(0/1 flags are booleans there).

### speech_to_text.py

- Subscribes `audio` (`audio_common_msgs/AudioData`, remapped to `speech_audio` by the launch file)
- Publishes `speech_to_text` (`speech_recognition_msgs/SpeechRecognitionCandidates`)
- Uses the action `sound_play` (`sound_play/SoundRequestAction`; ROS 2: `sound_play_msgs/action/SoundRequest`), only
  to know whether the robot is speaking (`~self_cancellation`).

| Name | Type | Default | Meaning |
|---|---|---|---|
| `~language` | string | `ja-JP` | Language of the recognition (the launch file sets `en-US`) |
| `~sample_rate` | int | `16000` | Sample rate of `audio` |
| `~sample_width` | int | `2` | Bytes per sample of `audio` |
| `~self_cancellation` | bool | `true` | Ignore the audio while one of `~tts_action_names` is active |
| `~tts_action_names` | string list | `['sound_play']` | Actions of text-to-speech |
| `~tts_tolerance` | double | `1.0` | Seconds after the end of speaking during which the audio is still ignored |

## Use cases

- [ros_speech_recognition](../ros_speech_recognition): more speech recognition engines
- [julius_ros](../julius_ros)

## Notes

### ROS 1 and ROS 2

Each script keeps its original rospy code (`RespeakerNode`, `SpeechToText`), used on ROS 1 without
[jsk_ros1_ros2_compat](https://github.com/jsk-ros-pkg/jsk_common/tree/master/jsk_ros1_ros2_compat) (jsk_common < 3.0),
next to a version written with jsk_ros1_ros2_compat (`RespeakerNodeCompat`, `SpeechToTextCompat`), used on ROS 2
and on ROS 1 with jsk_ros1_ros2_compat. `scripts/respeaker_gencfg.py` and the rostests are ROS 1 only.

### Regenerating `cfg/Respeaker.cfg` (ROS 1)

The configuration of dynamic_reconfigure is generated by reading the parameters from the device. To update it,
connect the device and run

```bash
rosrun respeaker_ros respeaker_gencfg.py
```

## Author

Yuki Furuta <<furushchev@jsk.imi.i.u-tokyo.ac.jp>>

## License

[Apache License](LICENSE)
