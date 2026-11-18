# emotion_analyzer

Services which estimate the emotions of a text or of a short voice with the streaming API of [Hume AI](https://www.hume.ai/).
`analyze_text` sends a text to the language model of Hume, `analyze_audio` sends a wave file or the last 2 seconds
of an `audio_common_msgs/AudioData` topic to its prosody and vocal burst models; both return the scores of the
emotions (e.g. `Joy`, `Surprise (positive)`) as a JSON string.

## Requirements

- An API key of Hume AI: sign up at https://platform.hume.ai/ and create a key in the settings of the platform
  (API keys). The analysis is done on the servers of Hume, so the node needs the internet.
- For the analysis of a live voice: a microphone and `audio_capture` (from `audio_common`) publishing 16 kHz,
  16 bit, mono wave data.
- Python package `hume[stream]` (the legacy `HumeStreamClient`, removed in hume 0.7). It is installed at build time:
  - ROS 1: into a virtualenv by `catkin_virtualenv` from `requirements.in` (`hume==0.4.2` on python 3.8,
    `hume==0.6.0` on python >= 3.9);
  - ROS 2: into a uv venv (`share/emotion_analyzer/venv`) from `pyproject.toml`/`uv.lock` by
    `jsk_ros1_ros2_compat_generate_uv_venv()`. Run `uv lock` after changing `pyproject.toml`.

## Usage

### ROS 2

```bash
colcon build --packages-up-to emotion_analyzer
source install/setup.bash
ros2 launch emotion_analyzer emotion_analyzer.launch.xml api_key:=<your_api_key>
```

Analyze a text:

```bash
ros2 service call /analyze_text emotion_analyzer/srv/AnalyzeText "{text: 'I am so happy to see you again'}"
```

Analyze a wave file of 5 seconds or shorter, e.g. the sample `data/purugacha_short.wav` (4 s) of the source tree
(`data/` is not installed; `data/purugacha.wav` is 8.8 s, too long):

```bash
ros2 service call /analyze_audio emotion_analyzer/srv/AnalyzeAudio \
  "{audio_file: '$HOME/ros2_ws/src/jsk_3rdparty/emotion_analyzer/data/purugacha_short.wav'}"
```

Analyze the voice of a microphone: start `audio_capture` in wave format and call the service with an empty
`audio_file`; the last 2 seconds of `/audio/audio` are analyzed.

```bash
ros2 launch emotion_analyzer sample_emotion_analyzer.launch.xml api_key:=<your_api_key> device:=plughw:1,0
ros2 service call /analyze_audio emotion_analyzer/srv/AnalyzeAudio "{audio_file: ''}"
```

`sample_emotion_analyzer.launch.xml` includes `capture_wave.launch.xml` of the ROS 2 `audio_capture`.

| Launch file | Arguments (default) |
|---|---|
| `emotion_analyzer.launch.xml` | `api_key` (`""`), `input_audio` (`/audio/audio`) |
| `sample_emotion_analyzer.launch.xml` | `api_key` (`""`), `device` (`""`, the ALSA device of `audio_capture`) |

### ROS 1

```bash
catkin build emotion_analyzer
source devel/setup.bash
roslaunch emotion_analyzer emotion_analyzer.launch api_key:=<your_api_key>
```

```bash
rosservice call /analyze_text "text: 'I am so happy to see you again'"
# data/ is in the source tree (rospack find gives it in a devel space)
rosservice call /analyze_audio "audio_file: '$(rospack find emotion_analyzer)/data/purugacha_short.wav'"
```

For the voice of a microphone:

```bash
roslaunch emotion_analyzer sample_emotion_analyzer.launch api_key:=<your_api_key>
# or start audio_capture yourself
roslaunch audio_capture capture.launch format:=wave device:=plughw:1,0
rosservice call /analyze_audio "audio_file: ''"
```

| Launch file | Arguments (default) |
|---|---|
| `emotion_analyzer.launch` | `api_key` (`""`), `input_audio` (`/audio/audio`) |
| `sample_emotion_analyzer.launch` | `api_key` (`""`); there is no `device` argument, `audio_capture` uses its default device |

### Microphone tips

- List the devices with `arecord -l`. Sometimes `hw` must be `plughw`, e.g. `device:=plughw:1,0`.
- When the device is busy, find the process with `fuser -v /dev/snd/*` and stop it.

## Interface

### analyze_text_service.py

Services:

- `/analyze_text` (`emotion_analyzer/AnalyzeText`): request `string text`, response `string result`, the JSON
  `{"emotions": [{"name": ..., "score": ...}, ...]}` of the first sentence (`LanguageConfig(granularity="sentence")`),
  or `{"error": "..."}`.

Parameters:

- `hume_api_key` (string, required): the API key of Hume AI. On ROS 1 it is the global parameter `/hume_api_key`
  set by the launch file; on ROS 2 a parameter of the node.

### analyze_audio_service.py

Subscribed topics:

- `~audio` (`audio_common_msgs/AudioData`): the raw audio, 16 kHz, 16 bit, mono (fixed in the code). Remapped to
  `input_audio` (`/audio/audio`) by the launch files.
- `/audio/audio_info` (`audio_common_msgs/AudioInfo`): only checked; a warning is printed unless `coding_format` is
  `wave`.

Services:

- `/analyze_audio` (`emotion_analyzer/AnalyzeAudio`): request `string audio_file`, response `string result`.
  A path of an audio file readable by `pydub`, or `''` for the last 2 seconds of `~audio`. Audio longer than
  5 seconds is refused. The result is the JSON `{"prosody": [...], "burst": [...]}` of the emotion scores
  (`null` when Hume found no speech or no vocal burst), or `{"error": "..."}`.

Parameters:

- `hume_api_key` (string, required): as above.
