# voicevox

ROS Interface for [VOICEVOX](https://voicevox.hiroshiba.jp/) (AI speech synthesis)

## Installation

Build this package. Don't forget to do `git submodule update --init` in this package.

```bash
cd /path/to/catkin_workspace
catkin build voicevox
```

## Usage

### Launch sound_play with VOICEVOX Text-to-Speech

```bash
roslaunch voicevox voicevox_texttospeech.launch
```

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
| 1 | 四国めたん-あまあま |
| 2 | ずんだもん-あまあま |
| 3 | 四国めたん-ノーマル |
| 4 | ずんだもん-ノーマル |
| 5 | 四国めたん-セクシー |
| 6 | ずんだもん-セクシー |
| 7 | 四国めたん-ツンツン |
| 8 | ずんだもん-ツンツン |
| 9 | 春日部つむぎ-ノーマル |
| 10 | 波音リツ-ノーマル |
| 11 | 雨晴はう-ノーマル |
| 12 | 玄野武宏-ノーマル |
| 13 | 白上虎太郎-ノーマル |
| 14 | 青山龍星-ノーマル |
| 15 | 冥鳴ひまり-ノーマル |
| 16 | 九州そら-あまあま |
| 17 | 九州そら-ノーマル |
| 18 | 九州そら-セクシー |
| 19 | 九州そら-ツンツン |
| 20 | 九州そら-ささやき |

#### For roseus users

```
$ roseus
(load "package://pr2eus/speak.l")

(ros::roseus "say_node")

(speak "JSKへようこそ。" :lang "波音リツ" :wait t :topic-name "robotsound_jp")
```
