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
| もち子さん | https://vtubermochio.wixsite.com/mochizora/利用規約 |
| 剣崎雌雄 | https://frontier.creatia.cc/fanclubs/413/posts/4507 |
| WhiteCUL | https://www.whitecul.com/guideline |
| 後鬼 | https://ついなちゃん.com/voicevox_terms/ |
| No.7 | https://voiceseven.com/#j0200 |
| ちび式じい | https://docs.google.com/presentation/d/1AcD8zXkfzKFf2ertHwWRwJuQXjNnijMxhz7AJzEkaI4 |
| 櫻歌ミコ | https://voicevox35miko.studio.site/rule |
| 小夜/SAYO | https://316soramegu.wixsite.com/sayo-official/guideline |
| ナースロボ＿タイプＴ | https://www.krnr.top/rules |
| †聖騎士 紅桜† | https://commons.nicovideo.jp/material/nc296132 |
| 雀松朱司 | https://virvoxproject.wixsite.com/official/voicevoxの利用規約 |
| 麒ヶ島宗麟 | https://virvoxproject.wixsite.com/official/voicevoxの利用規約 |
| 春歌ナナ | https://nanahira.jp/haruka_nana/guideline.html |
| 猫使アル | https://nekotukarb.wixsite.com/nekonohako/利用規約 |
| 猫使ビィ | https://nekotukarb.wixsite.com/nekonohako/利用規約 |
| 中国うさぎ | https://zunko.jp/con_ongen_kiyaku.html |
| 栗田まろん | https://aivoice.jp/character/maron/ |
| あいえるたん | https://www.infiniteloop.co.jp/special/iltan/terms/ |
| 満別花丸 | https://100hanamaru.wixsite.com/manbetsu-hanamaru/rule |
| 琴詠ニア | https://commons.nicovideo.jp/works/nc315435 |
| Voidoll | https://blog.nicovideo.jp/niconews/224589.html |
| 中部つるぎ | https://zunko.jp/con_ongen_kiyaku.html |
| ぞん子 | https://zonko.zone-energy.jp/guideline |

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
To get available voice name and speaker id, run following command
```bash
rosrun voicevox list_speaker.py
```

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
| 12 | 白上虎太郎-ふつう |
| 13 | 青山龍星-ノーマル |
| 14 | 冥鳴ひまり-ノーマル |
| 15 | 九州そら-あまあま |
| 16 | 九州そら-ノーマル |
| 17 | 九州そら-セクシー |
| 18 | 九州そら-ツンツン |
| 19 | 九州そら-ささやき |
| 20 | もち子さん-ノーマル |
| 21 | 剣崎雌雄-ノーマル |
| 22 | ずんだもん-ささやき |
| 23 | WhiteCUL-ノーマル |
| 24 | WhiteCUL-たのしい |
| 25 | WhiteCUL-かなしい |
| 26 | WhiteCUL-びえーん |
| 27 | 後鬼-人間ver. |
| 28 | 後鬼-ぬいぐるみver. |
| 29 | No.7-ノーマル |
| 30 | No.7-アナウンス |
| 31 | No.7-読み聞かせ |
| 32 | 白上虎太郎-わーい |
| 33 | 白上虎太郎-びくびく |
| 34 | 白上虎太郎-おこ |
| 35 | 白上虎太郎-びえーん |
| 36 | 四国めたん-ささやき |
| 37 | 四国めたん-ヒソヒソ |
| 38 | ずんだもん-ヒソヒソ |
| 39 | 玄野武宏-喜び |
| 40 | 玄野武宏-ツンギレ |
| 41 | 玄野武宏-悲しみ |
| 42 | ちび式じい-ノーマル |
| 43 | 櫻歌ミコ-ノーマル |
| 44 | 櫻歌ミコ-第二形態 |
| 45 | 櫻歌ミコ-ロリ |
| 46 | 小夜/SAYO-ノーマル |
| 47 | ナースロボ＿タイプＴ-ノーマル |
| 48 | ナースロボ＿タイプＴ-楽々 |
| 49 | ナースロボ＿タイプＴ-恐怖 |
| 50 | ナースロボ＿タイプＴ-内緒話 |
| 51 | †聖騎士 紅桜†-ノーマル |
| 52 | 雀松朱司-ノーマル |
| 53 | 麒ヶ島宗麟-ノーマル |
| 54 | 春歌ナナ-ノーマル |
| 55 | 猫使アル-ノーマル |
| 56 | 猫使アル-おちつき |
| 57 | 猫使アル-うきうき |
| 58 | 猫使ビィ-ノーマル |
| 59 | 猫使ビィ-おちつき |
| 60 | 猫使ビィ-人見知り |
| 61 | 中国うさぎ-ノーマル |
| 62 | 中国うさぎ-おどろき |
| 63 | 中国うさぎ-こわがり |
| 64 | 中国うさぎ-へろへろ |
| 65 | 波音リツ-クイーン |
| 66 | もち子さん-セクシー／あん子 |
| 67 | 栗田まろん-ノーマル |
| 68 | あいえるたん-ノーマル |
| 69 | 満別花丸-ノーマル |
| 70 | 満別花丸-元気 |
| 71 | 満別花丸-ささやき |
| 72 | 満別花丸-ぶりっ子 |
| 73 | 満別花丸-ボーイ |
| 74 | 琴詠ニア-ノーマル |
| 75 | ずんだもん-ヘロヘロ |
| 76 | ずんだもん-なみだめ |
| 77 | もち子さん-泣き |
| 78 | もち子さん-怒り |
| 79 | もち子さん-喜び |
| 80 | もち子さん-のんびり |
| 81 | 青山龍星-熱血 |
| 82 | 青山龍星-不機嫌 |
| 83 | 青山龍星-喜び |
| 84 | 青山龍星-しっとり |
| 85 | 青山龍星-かなしみ |
| 86 | 青山龍星-囁き |

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
