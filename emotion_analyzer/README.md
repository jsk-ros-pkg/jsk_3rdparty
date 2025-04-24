# Emotion Analyzer Service using Hume API (ROS1)

This ROS1 package provides a service to analyze emotions from a given text using the [Hume AI](https://www.hume.ai/) API.

## Requirements

- ROS1 Noetic
- Python 3.8+
- An API key from Hume AI

## Installation

Clone this repository and move to this directory

``` bash
rosdep install -iry --from-paths .
catkin build --this
```

then source your workspace

## Usage (Quick)

Using your microphone

``` bash
roslaunch emotion_analyzer sample_emotion_analyzer.launch api_key:=<your_api_key>
```


## Usage

### 1. Launch Emotion_Analyzer
```bash
roslaunch emotion_analyzer emotion_analyzer.launch api_key:=<your_api_key>
```

### 2. Call the service
For text,
```bash
rosservice call /analyze_text "text: '<text you want to analyze>'"
```
For prepared audio (up to 5 seconds),
```bash
rosservice call /analyze_audio "audio_file: <audio_file_path>"
```
As a sample, you can use `'/home/leus/ros/catkin_ws/src/jsk_3rdparty/emotion_analyzer/data/purugacha_short.wav'` as <audio_file_path>.

For audio from microphone,
```bash
roslaunch audio_capture capture.launch format:=wave
rosservice call /analyze_audio "audio_file: ''"
```
You can check the device information by `arecord -l`.
Sometimes you need to replace "hw" with "plughw": 
for example, `roslaunch audio_capture capture.launch format:=wave device:=plughw:1,0`.
When the device is busy, you can try `fuser -v /dev/snd/*` to get PID and kill it by `kill -9 <PID>`.







