julius_ros
==========

ROS Interface for Julius speech recognition engine

Please see
 - [Source](https://github.com/julius-speech/julius): Julius source code
 - [Japanese dictatoin kit](https://github.com/julius-speech/dictation-kit):
 - [Up-to-date doc](https://github.com/julius-speech/julius/tree/master/doc): New but incomplete documentation
 - [old doc](https://julius.osdn.jp/juliusbook/ja/): Old documentation

## Create Workspace

```bash
mkdir -p catkin_ws/src
cd  catkin_ws/src
wstool init .
wstool set --git jsk-ros-pkg/jsk_3rdparty https://github.com/jsk-ros-pkg/jsk_3rdparty.git -y
wstool update -t .
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install -y -r --from-paths . --ignore-src
cd ../
catkin build julius_ros
source devel/setup.bash
```

---

# GMM version

## Usage

``` bash
roslaunch julius_ros julius.launch
```

## Getting Recognition Results

``` bash
rostopic echo /speech_to_text/transcript[0]
かけ
---
たぬき
---
わかめ
---
```

## Service

``` bash
rosservice call /speech_recognition "vocabulary:
  words: ['みそ', 'しょうゆ', 'とんこつ']
"
# speak one word in the list above
results:
  transcript: ['\xe3\x81\xbf\xe3\x81\x9d', '\xe3\x81\x97\xe3\x82\x87\xe3\x81\x86\xe3\x82\x86', '\xe3\x81\xa8\xe3\x82\x93\xe3\x81\x93\xe3\x81\xa4']
    confidence: [1.0, 0.0, 0.0]
```

## Limitation (TODO)

- Only 'ひらがな' is supported for phoneme estimation.
- Only word list is supported.

## Author

Yuki Furuta <<furushchev@jsk.imi.i.u-tokyo.ac.jp>>


---

# DNN version

We use julius config file and DNN weights from [OSDN](https://osdn.net/frs/redir.php?m=ymu&f=%2Fjulius%2F66544%2Fdictation-kit-v4.4.zip).

For detail, please see [julius/Makefile.dictation-kit](https://github.com/jsk-ros-pkg/jsk_3rdparty/blob/master/3rdparty/julius/Makefile.dictation-kit)

## Usage

```bash
roslaunch julius_ros julius.launch dnn:=true
```

## Getting Recognition Results

```bash
rostopic echo --filter "print('transcript: [%s]\n---'%(', '.join(map(lambda x: '\'%s\''%(x.decode('utf-8')), m.transcript))))" /speech_to_text
transcript: [' こんにちは 。', ' こんにちは あ 。', ' 今日 わ 。', ' こんにちは は 。', ' 今日 は 。']
---
```

## Limitation
- `/audio` topic must be 1channel, 16bit, 16000Hz and wave format.
- DNN is computed on the CPU.
- CPU usage is about 2% during non-speech and about 100% during speech.
