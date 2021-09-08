# google_cloud_texttospeech

ROS Interface for Google Cloud Text-to-Speech

## Preparation

1. Register [Google Cloud Text-to-Speech API](https://cloud.google.com/text-to-speech)
2. Put the credential json for [Google Cloud Text-to-Speech API](https://cloud.google.com/text-to-speech) in your robot

## Installation

For python2.7, you need to use `google-cloud-texttospeech==1.0.1`.

```bash
sudo pip install google-cloud-texttospeech==1.0.1
```

## Usage

### Launch sound_play with Google Cloud Text-to-Speech

```bash
roslaunch google_cloud_texttospeech google_cloud_texttospeech.launch credential:=/your/credential/json/path
```

### Say something

#### For python users

```python
import rospy
from sound_play.libsoundplay import SoundClient

rospy.init_node('say_node')

client = SoundClient(sound_action='robotsound', sound_topic='robotsound')

client.say('hello!')
client.say('こんにちは', voice='ja')
```

You can change the voice by changing the voice_name.
Look at the following website for further details.
[Google Cloud Text-to-Speech Supported voices and languages](https://cloud.google.com/text-to-speech/docs/voices)


We can use standard and WaveNet voices.
WaveNet voices are higher quality voices with different pricing; in the list, they have the voice type 'WaveNet'.

You can also choose the language by specifying the language code.
`ja` for Japanese, `de` for German, etc.


```python
client.say('你好', voice='cmn-TW-Wavenet-A')
client.say('Hallo', voice='de')
```

#### For roseus users

```
$ roseus
(load "package://pr2eus/speak.l")

(ros::roseus "say_node")

(speak "JSKへようこそ。" :lang "ja-JP-Wavenet-B" :wait t)
(speak "Welcome to JSK." :lang "en-US-Wavenet-A" :wait t)
(speak "欢迎来到 JSK" :lang "cmn-TW-Wavenet-A" :wait t)
(speak "Willkommen bei JSK" :lang "de-DE-Wavenet-A" :wait t)
```

### Tips

By default, generated audio files are stored at `$HOME/.ros/google_cloud_texttospeech/cache`.
If you want to change the cache directory, please set `GOOGLE_CLOUD_TEXTTOSPEECH_CACHE_DIR` as an environment variable.

If you don't want to cache the files, please set `GOOGLE_CLOUD_TEXTTOSPEECH_CACHE_ENABLED=false`.
