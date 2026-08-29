# silero_vad_ros

This package provides VAD (Voice Activity Detection) code. It subscribes an audio topic and publish a flag if curretly speeched or not with VAD.
This package uses [silero-vad](https://github.com/snakers4/silero-vad).

## How to build

```bash
catkin build silero_vad_ros
```

## Example

Please make sure your PC has a microphone.
And then launch.

```bash
roslaunch silero_vad_ros sample.launch
```

And please talk to the microphone.
