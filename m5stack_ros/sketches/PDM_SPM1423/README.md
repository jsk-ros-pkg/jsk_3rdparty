# PDM SPM1423

Publish audio data and volume from [PDM_SPM1423](https://shop.m5stack.com/products/pdm-microphone-unit-spm1423) microphone as ROS topic via M5Stack

## Overview

Communication:

- PDM_SPM1423 <-(I2S)-> M5Stack <-(Bluetooth/Wi-Fi/USB)-> PC
- Connect M5Stack and PDM_SPM1423 microphone with Grove cable

Published topics:

- `/audio` (`audio_common_msgs/AudioData`)

  Audio stream. The format is wave, 2048 chunk size, 44100hz and 16bit depth.

- `/volume` (`std_msgs/Float32`)

  The volume of the audio.

## Usage

- Follow [README.md](https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/m5stack_ros)
  - We recommend Bluetooth or Wi-fi connection because USB connection is slow for some reason.

- Run

  ```bash
  roslaunch m5stack_ros pdm_spm1423.launch
  ```
