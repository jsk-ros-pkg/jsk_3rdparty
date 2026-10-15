# ros\_speech\_recognition

A ROS package for speech-to-text services.  
This package uses Python package [SpeechRecognition](https://pypi.python.org/pypi/SpeechRecognition) as a backend.

## Tutorials

### Normal tutorial

1. Install this package and SpeechReconition

  ```bash
  sudo apt install ros-${ROS_DISTRO}-ros-speech-recognition
  ```
  
2. Launch speech recognition node

  ```bash
  roslaunch ros_speech_recognition speech_recognition.launch
  ```
  
3. Echo `/speech_to_text`

  ```bash
  rostopic echo /speech_to_text
  # you can get the recognition result
  ```

### Parrotry tutorial

Parrotry mean オウム返し in Japanese

```bash
# english
roslaunch ros_speech_recognition parrotry.launch
# japanese
roslaunch ros_speech_recognition parrotry.launch language:=ja-JP
```

### Streaming recognition with Google Cloud (`GoogleCloudStream`)

In addition to the request/response `GoogleCloud` engine, a streaming engine
`GoogleCloudStream` is available. It uses the bidirectional gRPC
`StreamingRecognize` API and publishes interim hypotheses on
`~voice_interim_topic` while the user is still speaking, then publishes the
finalised utterance on `~voice_topic` as usual.

```bash
roslaunch ros_speech_recognition speech_recognition.launch \
    engine:=GoogleCloudStream \
    language:=ja-JP \
    google_cloud_credentials_json:=/path/to/credentials.json
```

Notes:

* Endpointing combines Google's own `is_final` flag with a stability timer
  controlled by `~google_cloud_endpoint_stable_s` (default: 1.0 s).
* The gRPC session has a 305 s server-side limit; the recognizer reconnects
  lazily on the next audio chunk, so long-running nodes are safe.
* When `~self_cancellation` is enabled, the streaming session is torn down
  while a `~tts_action_names` action is active and reopened automatically
  afterwards to avoid recognising the robot's own voice.

Example:

The following is an example of speech recognition using `GoogleCloudStream`.
`Interim:` shows intermediate recognition results, while `Result:` shows the final recognition result after the speaker finishes speaking.

```bash
[INFO] [1792069200.711352]: Interim: こんにちは。
[INFO] [1792069201.701424]: Result: こんにちは。
[INFO] [1792069218.021376]: Interim: 今日？
[INFO] [1792069218.196229]: Interim:  今日は！
[INFO] [1792069218.621582]: Interim: 今日はいい？
[INFO] [1792069218.710931]: Interim: 今日はいい天気。
[INFO] [1792069219.011066]: Interim: 今日はいい天気です。
[INFO] [1792069219.221464]: Interim: 今日はいい天気ですね。
[INFO] [1792069219.911446]: Interim: 今日はいい天気ですね。外
[INFO] [1792069220.090884]: Interim: 今日はいい天気ですね。外で。
[INFO] [1792069220.211304]: Interim: 今日はいい天気ですね。外で遊び。
[INFO] [1792069220.510955]: Interim: 今日はいい天気ですね。外で遊びませ。
[INFO] [1792069220.595351]: Interim: 今日はいい天気ですね。外で遊びません。
[INFO] [1792069220.814139]: Interim: 今日はいい天気ですね。外で遊びませんか？
[INFO] [1792069221.795355]: Result: 今日はいい天気ですね。外で遊びませんか？
[INFO] [1792069237.912371]: Interim:  ぜひ！
[INFO] [1792069238.089007]: Interim:  ぜひ 外！
[INFO] [1792069238.422502]: Interim:  ぜひ 外で！
[INFO] [1792069238.596863]: Interim:  ぜひ 外で遊び！
[INFO] [1792069238.691785]: Interim:  外で遊び！
[INFO] [1792069238.900729]: Interim: ぜひ 外で遊びましょう。
[INFO] [1792069239.021404]: Interim:  外で遊びましょう。
[INFO] [1792069239.503002]: Interim: ぜひ 外で遊びましょう。
[INFO] [1792069240.221812]: Result: ぜひ 外で遊びましょう。
```

## `speech_recognition_node.py` Interface

### Publishing Topics

* `~voice_topic` (`speech_recognition_msgs/SpeechRecognitionCandidates`)

  Speech recognition candidates topic name.

  Topic name is set by parameter  `~voice_topic`, and default value is `speech_to_text`.

* `~voice_interim_topic` (`std_msgs/String`)

  Interim (non-final) transcript published while the user is still speaking.
  Only valid when `~engine` is `GoogleCloudStream`.

  Topic name is set by parameter `~voice_interim_topic`, and default value is `speech_to_text/interim`.

* `sound_play` (`sound_play/SoundRequestAction`)

  Action client to play sound on events. If the action server is not available or `~enable_sound_effect` is `False`, no sound is played.
  
### Subscribing Topics

* `~audio_topic` (`audio_common_msgs/AudioData`)

  Audio stream data to be recognized.

  Topis name is set by parameter  `~audio_topic` and default value is `audio`.

### Advertising Services

* `speech_recognition` (`speech_recognition_msgs/SpeechRecognition`)

  Service for speech recognition

* `speech_recognition/start` (`std_srvs/Empty`)

  Start service for speech recognition

  This service is available when parameter `~contiunous` is `True`.

* `speech_recognition/start` (`std_srvs/Empty`)

  Stop service for speech recognition

  This service is available when parameter `~contiunous` is `True`.

## Parameters

* `~voice_topic` (`String`, default: `speech_to_text`)

  Publishing voice topic name

* `~audio_topic` (`String`, default: `audio`)

  Subscribing audio topic name

* `~enable_sound_effect` (`Bool`, default: `True`)

    Flag to enable or disable sound to play sound on recognition.

* `~language` (`String`, default: `en-US`)

  Language to be recognized
  
* `~engine` (`Enum[String]`, default: `Google`)

  Speech-to-text engine (To see full options use `dynamic_reconfigure`)
  
* `~energy_threshold` (`Double`, default: `300`)

  Threshold for Voice activity detection
  
* `~dynamic_energy_threshold` (`Bool`, default: `True`)

  Adaptive estimation for `energy_threshold`

* `~dynamic_energy_adjustment_damping` (`Double`, default: `0.15`)

  Damping threshold for dynamic VAD
  
* `~dynamic_energy_ratio` (`Double`, default: `1.5`)

  Energy ratio for dynamic VAD
  
* `~pause_threshold` (`Double`, default: `0.8`)

  Seconds of non-speaking audio before a phrase is considered complete
  
* `~operation_timeout` (`Double`, default: `0.0`)

  Seconds after an internal operation (e.g., an API request) starts before it times out
  
* `~listen_timeout` (`Double`, default: `0.0`)

  The maximum number of seconds that this will wait for a phrase to start before giving up
  
* `~phrase_time_limit` (`Double`, default: `10.0`)

  The maximum number of seconds that this will allow a phrase to continue before stopping and returning the part of the phrase processed before the time limit was reached
  
* `~phrase_threshold` (`Double`, default: `0.3`)

  Minimum seconds of speaking audio before we consider the speaking audio a phrase
  
* `~non_speaking_duration` (`Double`, default: `0.5`)

  Seconds of non-speaking audio to keep on both sides of the recording

* `~duration` (`Double`, default: `10.0`)

  Seconds of waiting for speech

* `~depth` (`Int`, default: `16`)

  Depth of audio signal
  
* `~n_channel` (`Int`, default: `1`)

  Total number of channels in audio data (e.g. 1: mono, 2: stereo)
  
* `~sample_rate` (`Int`, default: `16000`)

  Sample rate of audio signal
  
* `~buffer_size` (`Int`, default: `10240`)

  Maximum buffer size to store audio data for speech recognition
  
* `~start_signal` (`String`, default: `None`)

  Path to sound file for bell on the start of audio caption.

  If this is `None`, `/usr/share/sounds/freedesktop/stereo/bell.ogg` or `/usr/share/sounds/freedesktop/stereo/bell.oga` is used.
  
* `~recognized_signal` (`String`, default: `None`)

  Path to sound file for bell on the end of audio caption.

  If this is `None`, `/usr/share/sounds/freedesktop/stereo/message.ogg` or `/usr/share/sounds/freedesktop/stereo/message.oga` is used.
  
* `~success_signal` (`String`, default: `None`)

  Path to sound file for bell on getting successful recognition result.

  If this is `None`, `/usr/share/sounds/freedesktop/stereo/message-new-instant.ogg` or `/usr/share/sounds/freedesktop/stereo/message-new-instant.oga` is used.
  
* `~timeout_signal` (`String`, default: `None`)

  Path to sound file for bell on timeout for recognition.

  If this is `None`, `/usr/share/sounds/freedesktop/stereo/network-connectivity-lost.ogg` or `/usr/share/sounds/freedesktop/stereo/network-connectivity-lost.oga` is used.
  
* `~continuous` (`Bool`, default: False)

  Selecting to use topic or service. By default, service is used.

* `~auto_start` (`Bool`, default: True)

  Starting the speech recognition when launching.

* `~self_cancellation` (`Bool`, default: `True`)

  Whether the node recognize the sound heard when `~tts_action_names` is running or not.

  This options is for ignoring self voice sounds from recognition.

* `~tts_action_names` (`List[String]`, default: `['sound_play']`)

  Text-to-speech action name for self cancellation.

  The node ignores the voice heard when these Text-to-speech action is running.

* `~tts_tolerance` (`Float`, default: `1.0`)

   Tolerance seconds for self cancellation.

   The node ignores the voice with this tolerance seconds after `~tts_action_names` finish running.

* `~google_key` (`String`, default: `None`)

  Auth Key for Google API. If `None`, use public key. (No guarantee to be blocked.)  
  This is valid only if `~engine` is `Google`.
  
* `~google_cloud_credentials_json` (`String`, default: `None`)

  Path to credential json file. For JSK users, you can download from [Google Drive](https://drive.google.com/file/d/1VxniytpH9J12ii9jphtBylydY1_k5nXf/view?usp=sharing) link.
  This is valid if `~engine` is `GoogleCloud` or `GoogleCloudStream`.
  If the environment variable `GOOGLE_APPLICATION_CREDENTIALS` is already set, this parameter is ignored.

* `~google_cloud_preferred_phrases` (`[String]`, default: `None`)

  Preferred phrases parameters.
  This is valid only if `~engine` is `GoogleCloud`.

* `~google_cloud_model` (`String`, default: `""`)

  Recognition model name passed to the Google Cloud Speech API (e.g. `latest_long`, `latest_short`, `default`). Leave empty to use the API default.
  This is valid only if `~engine` is `GoogleCloudStream`.

* `~google_cloud_endpoint_stable_s` (`Double`, default: `1.0`)

  Seconds the interim transcript must remain unchanged before a result is treated as final, in addition to Google's own `is_final` signal. Lower values commit faster but may cut off slow speakers.
  This is valid only if `~engine` is `GoogleCloudStream`.

* `~log_interim_results` (`Bool`, default: `True`)

  If `True`, the interim transcript is also written to `rospy.loginfo` (deduplicated — only logged when the text actually changes). Set to `False` to silence the interim log line on noisy environments.
  This is valid only if `~engine` is `GoogleCloudStream`.
  
* `~bing_key` (`String`, default: `None`)

  Auth key for Bing API.  
  This is valid only if `~engine` is `bing`.

* `~vosk_model_path` (`String`, default: `None`)

  Path to trainded model for Vosk API.
  This is valid only if `~engine` is `Vosk`.

  If `en-US` or `ja` is selected as `~language`, you do not need to specify the path.
  To load other models, please download them from [Model list](https://alphacephei.com/vosk/models).
  
## Author

Yuki Furuta <<furushchev@jsk.imi.i.u-tokyo.ac.jp>>
