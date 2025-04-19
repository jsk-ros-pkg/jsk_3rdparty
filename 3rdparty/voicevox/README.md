# voicevox

ROS Interface for [VOICEVOX](https://voicevox.hiroshiba.jp/) (AI speech synthesis)

## TERM

[VOICEVOX](https://voicevox.hiroshiba.jp/) is basically free to use, but please check the terms of use below.

[TERM](https://voicevox.hiroshiba.jp/term)

Each voice synthesis character has its own rules. Please use this package according to those terms.

| Character name  |  term link |
| ---- | ---- |
| 四国めたん | https://zunko.jp/con_ongen_kiyaku.html |
| ずんだもん | https://zunko.jp/con_ongen_kiyaku.html |
| 春日部つむぎ | https://tsukushinyoki10.wixsite.com/ktsumugiofficial/利用規約 |
| 波音リツ | http://canon-voice.com/kiyaku.html |
| 雨晴はう | https://amehau.com/?page_id=225 |
| 玄野武宏 | https://virvoxproject.wixsite.com/official/voicevoxの利用規約 |
| 白上虎太郎 | https://virvoxproject.wixsite.com/official/voicevoxの利用規約 |
| 青山龍星 | https://virvoxproject.wixsite.com/official/voicevoxの利用規約 |
| 冥鳴ひまり | https://kotoran8zunzun.wixsite.com/my-site/利用規約 |
| 九州そら | https://zunko.jp/con_ongen_kiyaku.html |

## Installation

Build this package.

```bash
cd /path/to/catkin_workspace
catkin build voicevox
```

## Usage

### Launch sound_play with VOICEVOX Text-to-Speech

```bash
roslaunch voicevox voicevox_texttospeech.launch
```

<a id="saysomething"></a>
### Say something

#### For python users

```python
import rospy
from sound_play.libsoundplay import SoundClient

rospy.init_node('say_node')

client = SoundClient(sound_action='robotsound_jp', sound_topic='robotsound_jp')

client.say('こんにちは', voice='四国めたん-あまあま')
```

You can change the voice by changing the voice_name.
You can also specify the speaker id.
Look at the following tables for further details.

|  speaker_id  |  voice_name  |
| ---- | ---- |
| 0 | 四国めたん-あまあま |
| 1 | ずんだもん-あまあま |
| 2 | 四国めたん-ノーマル |
| 3 | ずんだもん-ノーマル |
| 4 | 四国めたん-セクシー |
| 5 | ずんだもん-セクシー |
| 6 | 四国めたん-ツンツン |
| 7 | ずんだもん-ツンツン |
| 8 | 春日部つむぎ-ノーマル |
| 9 | 波音リツ-ノーマル |
| 10 | 雨晴はう-ノーマル |
| 11 | 玄野武宏-ノーマル |
| 12 | 白上虎太郎-ノーマル |
| 13 | 青山龍星-ノーマル |
| 14 | 冥鳴ひまり-ノーマル |
| 15 | 九州そら-あまあま |
| 16 | 九州そら-ノーマル |
| 17 | 九州そら-セクシー |
| 18 | 九州そら-ツンツン |
| 19 | 九州そら-ささやき |

#### For roseus users

```
$ roseus
(load "package://pr2eus/speak.l")

(ros::roseus "say_node")

(speak "JSKへようこそ。" :lang "波音リツ" :wait t :topic-name "robotsound_jp")
```

### Tips

Normally, the server for speech synthesis starts up at `http://localhost:50021`.
You can change the url and port by setting values for `VOICEVOX_TEXTTOSPEECH_URL` and `VOICEVOX_TEXTTOSPEECH_PORT`.

You can also set the default character by setting `VOICEVOX_DEFAULT_SPEAKER_ID`.
Please refer to [here](#saysomething) for the speaker id.
