# aques_talk

ROS Interface for AqeusTalk2.

For detail, Please see [manual](https://www.a-quest.com/archive/manual/aqtk2_lnx_man.pdf) and [symbols table](https://www.a-quest.com/archive/manual/siyo_onseikigou.pdf)


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

All charactors except `[a-zA-Z0-9ぁ-んァ-ンー、。?？]` and Kanji are removed so that AquesTalk2 can recognize them.

```
’こ”^＾ん/に*|ち＊＊｜は；：;:『』！#＃$＄&＆~〜()（）＿_｀\_\\\_・・<\>＊｀＞＜+＋@＠ー=＝%[]-￥
->
こんにちはー
```

**Do not** input the unpronounceable or control characters. For example,
- `ヰ` `ヱ`
- `ぁ` `〜` at the beginning of sentence (`ぁ` or `〜` at the beginning of sentence cannot be pronounced)
- `たぁ` (On the other hand, `ふぁ` can be pronounced.)
- `'` `"` in many programs (Because many programs recognize them as string control charactor)
- `!` `` ` `` if you use shell (Because shell recognizes them as control charactor)

You should be careful in input charactors about how they are pronounced.
```
Wrong -> Correct
20時です -> 20じです # Kanji is sometimes mispronounced.
fetch15 -> フェッチ15 # fetch is pronounced as エフ、イー、ティー、シー、エイチ
73B2 -> 7,3,B,2 # 73B2 is pronounced as ななじゅうさんビーに
私の名前は -> 私の名前わ
```
