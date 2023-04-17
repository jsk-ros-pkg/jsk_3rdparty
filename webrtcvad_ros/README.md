# webrtcvad_ros

This package provides VAD (Voice Activity Detection) code. It subscribes an audio topic and publish a flag if curretly speeched or not with VAD.
There is 2 types of implementation.

1. One uses [webrtcvad](https://github.com/wiseman/py-webrtcvad).
2. One uses [silero-vad](https://github.com/snakers4/silero-vad/tree/master/examples/microphone_and_webRTC_integration)

## Prerequities

```bash
pip install webrtcvad
```

## Example

```bash
roslaunch webrtcvad_ros sample.launch
```
