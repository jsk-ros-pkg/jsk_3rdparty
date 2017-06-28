julius_ros
==========

ROS Interface for Julius speech recognition engine

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
