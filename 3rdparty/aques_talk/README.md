# aques_talk

ROS Interface for AqeusTalk2

## Usage

### Launch sound_play with AquesTalk2 Text-to-Speech

```bash
roslaunch aques_talk aques_talk.launch
```

### Say something

```bash
$ rostopic pub /robotsound_jp  sound_play/SoundRequest "{sound: -3, command: 1, volume: 10.0, arg: 'こんにちわ', arg2: ''}"
$ rostopic pub /robotsound_jp  sound_play/SoundRequest "{sound: -3, command: 1, volume: 10.0, arg: 'こんにちわピーアールツー', arg2: ''}"
```

```python
import rospy
from sound_play.libsoundplay import SoundClient

rospy.init_node('say_node')

client = SoundClient(sound_action='robotsound_jp', sound_topic='robotsound_jp')

client.say('こんにちは')

# note that
client.say('Hello')
# does not work ! Use
client.say('Hello', voice='en')


```


### Limitations on input strings
```
Wrong -> Correct
20時です -> 20じです # Kanji is sometimes mispronounced.
fetch15 -> フェッチ15 # fetch is pronounced as エフ、イー、ティー、シー、エイチ
73B2 -> 7,3,B,2 # 73B2 is pronounced as ななじゅうさんビーに
私の名前は -> 私の名前わ
```
