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
  
## Interface

### Publishing Topics

* `sound_play` (`sound_play/SoundRequestAction`)

  Action client to play sound on events. If the action server is not available or `~enable_sound_effect` is `False`, no sound is played.
  
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
