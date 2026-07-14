# webrtcvad_ros

This package provides VAD (Voice Activity Detection) code. It subscribes an audio topic and publish a flag if curretly speeched or not with VAD.
This package uses [webrtcvad](https://github.com/wiseman/py-webrtcvad).

## Prerequities

```bash
pip install webrtcvad
```

## Example

```bash
roslaunch webrtcvad_ros sample.launch
```
