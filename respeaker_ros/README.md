respeaker_ros
=============

A ROS Package for Respeaker Mic Array


## Supported Devices

- [Respeaker Mic Array v2.0](http://wiki.seeedstudio.com/ReSpeaker_Mic_Array_v2.0/)

    ![Respeaker Mic Array v2.0](https://github.com/SeeedDocument/ReSpeaker_Mic_Array_V2/raw/master/img/Hardware%20Overview.png)

## Preparation

1. Install this package

    Assumed that ROS is properly installed.

    You can install this package via `apt-get`.

    ```bash
    sudo apt-get install ros-$ROS_DISTRO-respeaker-ros
    ```

    Or you can also build from the source.

    ```bash
    mkdir -p ~/catkin_ws/src && ~/catkin_ws/src
    git clone https://github.com/jsk-ros-pkg/jsk_3rdparty.git
    cd ~/catkin_ws
    source /opt/ros/kinetic/setup.bash
    rosdep install --from-paths src -i -r -n -y
    catkin config --init
    catkin build respeaker_ros
    source ~/catkin_ws/devel/setup.bash
    ```

1. Register respeaker udev rules

    Normally, we cannot access USB device without permission from user space.
    Using `udev`, we can give the right permission on only respeaker device automatically.

    Please run the command as followings to install setting file:

    ```bash
    roscd respeaker_ros
    sudo cp -f $(rospack find respeaker_ros)/config/60-respeaker.rules /etc/udev/rules.d/60-respeaker.rules
    sudo systemctl restart udev
    ```

    And then re-connect the device.

1. Update firmware

    ```bash
    git clone https://github.com/respeaker/usb_4_mic_array.git
    cd usb_4_mic_array
    sudo python dfu.py --download 6_channels_firmware.bin  # The 6 channels version 
    ```

## How to use

1. Run executables

    ```bash
    roslaunch respeaker_ros sample_respeaker.launch
    rostopic echo /sound_direction     # Result of DoA
    rostopic echo /sound_localization  # Result of DoA as Pose
    rostopic echo /is_speeching        # Result of VAD
    rostopic echo /audio               # Raw audio
    rostopic echo /speech_audio        # Audio data while speeching
    ```

    You can also set various parameters via `dynamic_reconfigure`.

    ```bash
    sudo apt install ros-kinetic-rqt-reconfigure  # Install if not
    rosrun rqt_reconfigure rqt_reconfigure
    ```

    To set LED color, publish desired color:

    ```bash
    rostopic pub /status_led std_msgs/ColorRGBA "r: 0.0
    g: 0.0
    b: 1.0
    a: 0.3"
    ```

## Parameters for respeaker_node.py

  - ### Publishing topics

    - `audio` (`audio_common_msgs/AudioData`)

      Processed audio for ASR. 1 channel.

    - `audio_info` (`audio_common_msgs/AudioInfo`)

      Audio info with respect to `~audio`.

    - `audio_raw` (`audio_common_msgs/AudioData`)

      Micarray audio data has 4-channels. Maybe you need to update respeaker firmware.

      If the firmware isn't supported, this will not be output.

    - `audio_info_raw` (`audio_common_msgs/AudioInfo`)

      Audio info with respect to `~audio_raw`.

      If the firmware isn't supported, this will not be output.

    - `speech_audio` (`audio_common_msgs/AudioData`)

      Audio data while a person is speaking using the VAD function.

    - `speech_audio_raw` (`audio_common_msgs/AudioData`)

      Audio data has 4-channels while a person is speaking using the VAD function.

      If the firmware isn't supported, this will not be output.

    - `audio_merged_playback` (`audio_common_msgs/AudioData`)

      Data that combines the sound of mic and speaker.

      If the firmware isn't supported, this will not be output.

      For more detail, please see https://wiki.seeedstudio.com/ReSpeaker_Mic_Array_v2.0/

    - `~is_speeching` (`std_msgs/Bool`)

      Using VAD function, publish whether someone is speaking.

    - `~sound_direction` (`std_msgs/Int32`)

      Direction of sound.

    - `~sound_localization` (`geometry_msgs/PoseStamped`)

      Localized Sound Direction. The value of the position in the estimated direction with `~doa_offset` as the radius is obtained.

  - ### Parameters

    - `~update_rate` (`Double`, default: `10.0`)

      Publishing info data such as `~is_speeching`, `~sound_direction`, `~sound_localization`, `~speech_audio` and `~speech_audio_raw`.

    - `~sensor_frame_id` (`String`, default: `respeaker_base`)

      Frame id.

    - `~doa_xy_offset` (`Double`, default: `0.0`)

      `~doa_offset` is a estimated sound direction's radius.

    - `~doa_yaw_offset` (`Double`, default: `90.0`)

      Estimated DoA angle offset.

    - `~speech_prefetch` (`Double`, default: `0.5`)

      Time to represent how long speech is pre-stored in buffer.

    - `~speech_continuation` (`Double`, default: `0.5`)

      If the time between the current time and the time when the speech is stopped is shorter than this time,
      it is assumed that someone is speaking.

    - `~speech_max_duration` (`Double`, default: `7.0`)

    - `~speech_min_duration` (`Double`, default: `0.1`)

       If the speaking interval is within these times, `~speech_audio` and `~speech_audio_raw` will be published.

    - `~suppress_pyaudio_error` (`Bool`, default: `True`)

      If this value is `True`, suppress error from pyaudio.

## Use cases

### Voice Recognition

- [ros_speech_recognition](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/ros_speech_recognition)
- [julius_ros](http://wiki.ros.org/julius_ros)

## Notes

The configuration file for `dynamic_reconfigure` in this package is created automatically by reading the parameters from devices.
Though it will be rare case, the configuration file can be updated as followings:

1. Connect the device to the computer.
1. Run the generator script.

    ```bash
    rosrun  respeaker_ros respeaker_gencfg.py
    ```
1. You will see the updated configuration file at `$(rospack find respeaker_ros)/cfg/Respeaker.cfg`.


## Author

Yuki Furuta <<furushchev@jsk.imi.i.u-tokyo.ac.jp>>

## License

[Apache License](LICENSE)
