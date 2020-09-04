# google_cloud_texttospeech

ROS Interface for Google Cloud Text-to-Speech

## Preparation

1. Register [Google Cloud Text-to-Speech API](https://cloud.google.com/text-to-speech)
2. Put the credentail json for [Google Cloud Text-to-Speech API](https://cloud.google.com/text-to-speech) in your robot

## Installation

For python2.7, you need to use `google-cloud-texttospeech==1.0.1`.

```bash
sudo pip install google-cloud-texttospeech==1.0.1
```

## Usage

### Launch sound_play with Google Cloud Text-to-Speech

```bash
roslaunch google_cloud_texttospeech google_cloud_texttospeech.launch credentail:=/your/credentail/json/path
```

### Say something

```bash
# english
rosrun sound_play say.py 'Hello' dummy 1.0 robotsound:=robotsound
# japanese
rosrun sound_play say.py 'こんにちは' dummy 1.0 robotsound:=robotsound_jp
```
