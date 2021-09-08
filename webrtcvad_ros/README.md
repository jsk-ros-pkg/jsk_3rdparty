# webrtcvad_ros

This package provides a wrapper node for [webrtcvad](https://github.com/wiseman/py-webrtcvad). It subscribes an audio topic and publish a flag if curretly speeched or not with VAD.

## Prerequities

```bash
pip install webrtcvad
```

## Example

```bash
roslaunch webrtcvad_ros sample.launch
```
