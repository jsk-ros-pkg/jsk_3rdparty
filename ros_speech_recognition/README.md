ros_speech_recognition
======================

A ROS package for speech-to-text services.  
This package uses Python package [SpeechRecognition](https://pypi.python.org/pypi/SpeechRecognition) as a backend.

## Tutorials

1. Install this package and SpeechReconition

  ```bash
  sudo apt install ros-${ROS_DISTRO}-ros-speech-recognition
  ```
  
2. Launch speech recognition node

  ```bash
  roslaunch ros_speech_recognition speech_recognition.launch
  ```
  
3. Use from Python

  ```python
  import rospy
  from ros_speech_recognition import SpeechRecognitionClient
  
  rospy.init_node("client")
  client = SpeechRecognitionClient()
  result = client.recognize()  # Please say 'Hello, world!' towards microphone
  print result # => 'Hello, world!'
  ```
  
If you are using `ros_speech_recognition` with `~continuous` is `True`, you can subscribe `/Tablet/voice` (`speech_recognition_msgs/SpeechRecognitionCandidates`) message.

1. Launch sample launch file.


    ```bash
    roslaunch ros_speech_recognition sample_ros_speech_recognition.launch
    ```

2. echo the message.


    ```bash
    $ rostopic echo /Tablet/voice
    transcript:
      - may I help you
    confidence: [0.9286448955535889]
    sentences:
      -
        header:
          seq: 0
          stamp:
            secs: 1641425262
            nsecs: 268165588
          frame_id: ''
        words:
          -
            start_time: 0.0
            end_time: 0.2
            word: "may"
            confidence: 0.91376436
            speaker_tag: 0
          -
            start_time: 0.2
            end_time: 0.4
            word: "I"
            confidence: 0.9366196
            speaker_tag: 0
          -
            start_time: 0.4
            end_time: 0.5
            word: "help"
            confidence: 0.9531065
            speaker_tag: 0
          -
            start_time: 0.5
            end_time: 0.8
            word: "you"
            confidence: 0.9110889
            speaker_tag: 0
    ---
    transcript:
      - pick up the red kettle
    confidence: [0.9499567747116089]
    sentences:
      -
        header:
          seq: 0
          stamp:
            secs: 1641425268
            nsecs:  58182954
          frame_id: ''
        words:
          -
            start_time: 0.0
            end_time: 0.4
            word: "pick"
            confidence: 0.953269
            speaker_tag: 0
          -
            start_time: 0.4
            end_time: 0.6
            word: "up"
            confidence: 0.95326656
            speaker_tag: 0
          -
            start_time: 0.6
            end_time: 0.8
            word: "the"
            confidence: 0.96866167
            speaker_tag: 0
          -
            start_time: 0.8
            end_time: 1.1
            word: "red"
            confidence: 0.98762906
            speaker_tag: 0
          -
            start_time: 1.1
            end_time: 1.5
            word: "kettle"
            confidence: 0.8869578
            speaker_tag: 0
      ```

The `word` is recognized word and the `confidence` means a higher number indicates an estimated greater likelihood that the recognized words are correct.
`start_time` indicates time offset relative to the beginning of the audio (timestamp of header), and corresponding to the start of the spoken word.
`end_time` indicates time offset relative to the beginning of the audio, and corresponding to the end of the spoken word.


## Interface

### Publishing Topics

* `sound_play` (`sound_play/SoundRequestAction`)

  Action client to play sound on events. If the action server is not available or `~enable_sound_effect` is `False`, no sound is played.
  

* `/Tablet/voice` (`speech_recognition_msgs/SpeechRecognitionCandidates`)

    Publish recognized results when `~continuous` is `True`.

### Subscribing Topics

* `audio` (`audio_common_msgs/AudioData`)

  Audio stream data to be recognized.

### Advertising Services

* `speech_recognition` (`speech_recognition_msgs/SpeechRecognition`)

  Service for speech recognition

## Parameters

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

* `~audio_topic` (`String`, default: `audio`)

  Topic name of input audio data
  
* `~depth` (`Int`, default: `16`)

  Depth of audio signal
  
* `~n_channel` (`Int`, default: `1`)

  Total number of channels in audio data (e.g. 1: mono, 2: stereo)
  
* `~sample_rate` (`Int`, default: `16000`)

  Sample rate of audio signal
  
* `~buffer_size` (`Int`, default: `10240`)

  Maximum buffer size to store audio data for speech recognition
  
* `~start_signal` (`String`, default: `/usr/share/sounds/ubuntu/stereo/bell.ogg`)

  Path to sound file for bell on the start of audio caption
  
* `~recognized_signal` (`String`, default: `/usr/share/sounds/ubuntu/stereo/button-toggle-on.ogg`)

  Path to sound file for bell on the end of audio caption
  
* `~success_signal` (`String`, default: `/usr/share/sounds/ubuntu/stereo/message-new-instant.ogg`)

  Path to sound file for bell on getting successful recognition result
  
* `~timeout_signal` (`String`, default: `/usr/share/sounds/ubuntu/stereo/window-slide.ogg`)

  Path to sound file for bell on timeout for recognition
  
* `~continuous` (`Bool`, default: False)

  Selecting to use topic or service. By default, service is used.

* `~google_key` (`String`, default: `None`)

  Auth Key for Google API. If `None`, use public key. (No guarantee to be blocked.)  
  This is valid only if `~engine` is `Google`.
  
* `~google_cloud_credentials_json` (`String`, default: `None`)

  Path to credential json file.
  This is valid only if `~engine` is `GoogleCloud`.
  
* `~google_cloud_preferred_phrases` (`[String]`, default: `None`)

  Preferred phrases parameters.
  This is valid only if `~engine` is `GoogleCloud`.
  
* `~bing_key` (`String`, default: `None`)

  Auth key for Bing API.  
  This is valid only if `~engine` is `bing`.
  
## Author

Yuki Furuta <<furushchev@jsk.imi.i.u-tokyo.ac.jp>>
