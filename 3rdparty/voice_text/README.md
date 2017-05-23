voice_text
==========

ROS Interface for HOYA VoiceText Speech Synthesis Engine

## Installation

1. Install Library

```bash
cd /usr
sudo tar xf /path/to/vt.tar
```

2. Put License

```bash
sudo cp /path/to/verification.txt /usr/vt/sayaka/M16/data-common/verify/
```

3. Build this package

```bash
cd /path/to/catkin_workspace
catkin build voice_text
```

## Usage

1. Launch `sound_play` node

```bash
roslaunch voice_text voice_text.launch
```

2. Say

Now speech synthesis is enabled.

```bash
rosrun sound_play say.py Hello!
```

Robot says "Hello!"
