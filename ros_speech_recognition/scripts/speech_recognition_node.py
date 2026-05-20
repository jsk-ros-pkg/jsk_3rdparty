#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

from ros_speech_recognition.compat import ros_compat
from ros_speech_recognition.compat import ROS_VERSION
import speech_recognition as SR
from ros_speech_recognition.recognize_google_cloud import RecognizerEx
from ros_speech_recognition.streaming_google_cloud import GoogleCloudStreamingRecognizer
import ros_speech_recognition.recognize_vosk
import json
import array
import os
import sys
from threading import Lock
from threading import Thread
from threading import Event

import numpy as np

from audio_common_msgs.msg import AudioData
from std_msgs.msg import String
if ROS_VERSION == 2:
    from sound_play_msgs.msg import SoundRequest
    from sound_play_msgs.action import SoundRequest as SoundRequestAction
    SoundRequestGoal = SoundRequestAction.Goal
    from action_msgs.msg import GoalStatus, GoalStatusArray
    # status topic of an action and state of an executing goal
    TTS_STATUS_TOPIC, TTS_STATUS_ACTIVE = '/_action/status', GoalStatus.STATUS_EXECUTING
    # rclpy requires a depth (ros_compat's default 1 may drop audio)
    AUDIO_QUEUE_SIZE = 10
else:
    from sound_play.msg import SoundRequest, SoundRequestAction, SoundRequestGoal
    from actionlib_msgs.msg import GoalStatus, GoalStatusArray
    TTS_STATUS_TOPIC, TTS_STATUS_ACTIVE = '/status', GoalStatus.ACTIVE
    AUDIO_QUEUE_SIZE = None
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from speech_recognition_msgs.srv import SpeechRecognition
from std_srvs.srv import Empty

from ros_speech_recognition.cfg import SpeechRecognitionConfig as Config


if hasattr(array.array, 'tobytes'):
    def array_to_bytes(arr):
        return arr.tobytes()
else:
    # Python 2.7
    def array_to_bytes(arr):
        return arr.tostring()


def resolve_sound_signal_path(path):
    if os.path.exists(path):
        return path
    attempted_paths = [path]
    base, ext = os.path.splitext(path)
    if ext == ".ogg":
        oga_path = base + ".oga"
        if os.path.exists(oga_path):
            return oga_path
        attempted_paths.append(oga_path)
    ros_compat.logwarn(None,
        "Sound signal file not found on the machine "
        "where speech_recognition is running: {}".format(
            " or ".join(attempted_paths))
    )
    return path


class ROSAudio(SR.AudioSource):
    def __init__(self, topic_name="audio", depth=16, n_channel=1,
                 sample_rate=16000, chunk_size=1024, buffer_size=10240,
                 node=None):
        assert buffer_size > chunk_size

        self.node = node
        self.topic_name = topic_name
        self.buffer_size = buffer_size

        if depth == 8:
            self.SAMPLE_WIDTH = 1
        elif depth == 16:
            self.SAMPLE_WIDTH = 2
        elif depth == 32:
            self.SAMPLE_WIDTH = 4
        else:
            raise ValueError("depth must be 8, 16 or 32")

        self.SAMPLE_RATE = sample_rate
        self.CHUNK = chunk_size
        self.n_channel = n_channel

        self.stream = None

    def open(self):
        if self.stream is not None:
            self.stream.close()
            self.stream = None
        self.stream = ROSAudio.AudioStream(
            self.topic_name, self.buffer_size, depth=self.SAMPLE_WIDTH*8,
            n_channel=self.n_channel, node=self.node)
        return self

    def close(self):
        self.stream.close()
        self.stream = None

    def __enter__(self):
        return self.open()

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    class AudioStream(object):
        def __init__(self, topic_name, buffer_size=10240, depth=16,
                     n_channel=1, target_channel=0, node=None):
            self.node = node
            self.buffer_size = buffer_size
            self.lock = Lock()
            self.buffer = bytes()
            self.depth = depth
            self.n_channel = n_channel
            self.target_channel = min(self.n_channel - 1, max(0, target_channel))
            self.sub_audio = ros_compat.create_subscription(node,
                topic_name, AudioData, self.audio_cb, queue_size=AUDIO_QUEUE_SIZE)
            self.type_code = {}
            for code in ['b', 'h', 'i', 'l']:
                self.type_code[array.array(code).itemsize] = code

        def read_once(self, size):
            with self.lock:
                buf = self.buffer[:size]
                self.buffer = self.buffer[size:]
                return buf

        def read(self, size):
            while not ros_compat.is_shutdown(self.node) and len(self.buffer) < size:
                ros_compat.sleep(self.node, 0.001)
            return self.read_once(size)

        def close(self):
            try:
                if ROS_VERSION == 2:
                    self.node.destroy_subscription(self.sub_audio)
                else:
                    self.sub_audio.unregister()
            except:
                pass
            self.buffer = bytes()

        def audio_cb(self, msg):
            with self.lock:
                dtype = self.type_code[self.depth/8]
                # take out target_channel channel data from multi channel data
                data = array.array(dtype, bytes(msg.data)).tolist()
                chan_data = data[self.target_channel::self.n_channel]
                self.buffer += array_to_bytes(array.array(dtype, chan_data))
                overflow = len(self.buffer) - self.buffer_size
                if overflow > 0:
                    self.buffer = self.buffer[overflow:]


class ROSSpeechRecognition(object):
    def __init__(self, node=None):
        self.node = node
        self.default_duration = ros_compat.get_param(node, "~duration", 10.0)
        self.engine = None
        self.recognizer = RecognizerEx() # Use custom Recognizer to support exteded API
        self.audio = ROSAudio(topic_name=ros_compat.get_param(node, "~audio_topic", "audio"),
                              depth=ros_compat.get_param(node, "~depth", 16),
                              n_channel=ros_compat.get_param(node, "~n_channel", 1),
                              sample_rate=ros_compat.get_param(node, "~sample_rate", 16000),
                              buffer_size=ros_compat.get_param(node, "~buffer_size", 10240),
                              node=node)

        # initialize sound play client
        if ros_compat.get_param(node, '~enable_sound_effect', True):
            self.act_sound = ros_compat.create_simple_action_client(node,
                "sound_play", SoundRequestAction)
            if not self.act_sound.wait_for_server(5.0):
                ros_compat.logwarn(node, "Failed to find sound_play action. "
                              "Disabled sound effect on recognition.")
                self.act_sound = None
        else:
            self.act_sound = None
        self.signals = {
            "start": ros_compat.get_param(node, "~start_signal", None),
            "recognized": ros_compat.get_param(node, "~recognized_signal", None),
            "success": ros_compat.get_param(node, "~success_signal", None),
            "timeout": ros_compat.get_param(node, "~timeout_signal", None),
        }
        default_signals = {
            "start": "/usr/share/sounds/freedesktop/stereo/bell.ogg",
            "recognized": "/usr/share/sounds/freedesktop/stereo/message.ogg",
            "success": "/usr/share/sounds/freedesktop/stereo/message-new-instant.ogg",
            "timeout": "/usr/share/sounds/freedesktop/stereo/network-connectivity-lost.ogg",
        }
        if self.act_sound is not None:
            for key in self.signals.keys():
                if self.signals[key] is None:
                    self.signals[key] = resolve_sound_signal_path(
                        default_signals[key])

        # ignore voice input while the robot is speaking
        self.self_cancellation = ros_compat.get_param(node, "~self_cancellation", True)
        # time to assume as SPEAKING after tts service is finished
        self.tts_tolerance = ros_compat.get_param(node, "~tts_tolerance", 1.0)  # sec
        tts_action_names = ros_compat.get_param(node,
            '~tts_action_names', ['sound_play'])

        self.tts_action = None
        self.last_tts = None
        self.is_canceling = False
        self.tts_actions = []
        # the goal states of tts_actions come from their status topics, since
        # ROS2 action clients do not keep the goals of other clients
        # (actionlib's action_client.last_status_msg)
        self.tts_status = {}
        if self.self_cancellation:
            for tts_action_name in tts_action_names:
                tts_action = ros_compat.create_simple_action_client(node,
                    tts_action_name, SoundRequestAction)
                if tts_action.wait_for_server(5.0):
                    self.tts_actions.append(tts_action)
                    ros_compat.create_subscription(node,
                        tts_action_name + TTS_STATUS_TOPIC, GoalStatusArray,
                        lambda msg, a=tts_action: self.tts_status.__setitem__(a, msg),
                        queue_size=1, latch=True)
                else:
                    ros_compat.logerr(node,
                        "action '{}' is not initialized."
                        .format(tts_action_name))
                self.tts_timer = ros_compat.create_timer(node, 0.1, self.tts_timer_cb)

        # GoogleCloudStream backend state — lazy-initialised in config_callback.
        # These MUST be set before dyn_srv = Server(...) because Server.__init__
        # synchronously fires config_callback once, and that callback reads
        # self._stream_recognizer.
        self._stream_recognizer = None
        self._stream_thread = None
        self._stream_stop_evt = None
        self.pub_interim = None
        self._log_interim = rospy.get_param("~log_interim_results", True)

        # dynamic_reconfigure (ROS1) or ROS2 parameters with Config.defaults
        # (cfg/ros2/SpeechRecognitionConfig.py)
        self.dyn_srv = ros_compat.create_reconfigure_server(node,
            Config, self.config_callback, Config.defaults)

        self.stop_fn = None
        self.continuous = ros_compat.get_param(node, "~continuous", False)
        self.auto_start = ros_compat.get_param(node, "~auto_start", True)
        self.enable_audio_cb = self.auto_start
        if self.continuous:
            ros_compat.loginfo(node, "Enabled continuous mode")
            ros_compat.loginfo(node, "Auto start: {}".format(self.auto_start))
            self.pub = ros_compat.create_publisher(node, ros_compat.get_param(node, "~voice_topic", "speech_to_text"),
                                       SpeechRecognitionCandidates,
                                       queue_size=1)
            self.pub_interim = rospy.Publisher(
                rospy.get_param("~voice_interim_topic", "speech_to_text/interim"),
                String, queue_size=10)
            self.start_srv = ros_compat.create_service(node,
                "speech_recognition/start",
                Empty, self.speech_recogniton_start_srv_cb)
            self.stop_srv = ros_compat.create_service(node,
                "speech_recognition/stop",
                Empty, self.speech_recogniton_stop_srv_cb)
        else:
            ros_compat.loginfo(node, "Disabled continuous mode, waiting for speech_recognition service")
            self.srv = ros_compat.create_service(node, "speech_recognition",
                                     SpeechRecognition,
                                     self.speech_recognition_srv_cb)

    def config_callback(self, config, level):
        # config for engine
        self.language = config.language
        if self.engine != config.engine:
            self.args = {}
            self.engine = config.engine
            # Lazy-build the streaming GCP wrapper when first selected.
            if (self.engine == Config.SpeechRecognition_GoogleCloudStream
                    and self._stream_recognizer is None):
                creds = rospy.get_param("~google_cloud_credentials_json", None)
                if creds and not os.environ.get("GOOGLE_APPLICATION_CREDENTIALS"):
                    os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = creds
                self._stream_recognizer = GoogleCloudStreamingRecognizer(
                    sample_rate=self.audio.SAMPLE_RATE,
                    language_code=self.language,
                    model=rospy.get_param("~google_cloud_model", ""),
                    endpoint_stable_s=rospy.get_param(
                        "~google_cloud_endpoint_stable_s", 1.0),
                )
                rospy.loginfo("GoogleCloudStream recognizer ready (lang=%s)",
                              self.language)

        # config for adaptive thresholding
        self.dynamic_energy_threshold = config.dynamic_energy_threshold
        # also stop the recognizer's own adjustment in listen() when disabled,
        # which was always enabled before (from Michi-Tsubaki's ROS2 port)
        self.recognizer.dynamic_energy_threshold = self.dynamic_energy_threshold
        if self.dynamic_energy_threshold:
            config.energy_threshold = self.recognizer.energy_threshold
        else:
            self.recognizer.energy_threshold = config.energy_threshold
        self.recognizer.dynamic_energy_adjustment_damping = config.dynamic_energy_adjustment_damping
        self.recognizer.dynamic_energy_ratio = config.dynamic_energy_ratio

        # config for timeout
        if config.listen_timeout > 0.0:
            self.listen_timeout = config.listen_timeout
        else:
            self.listen_timeout = None
        if config.phrase_time_limit > 0.0:
            self.phrase_time_limit = config.phrase_time_limit
        else:
            self.phrase_time_limit = None
        if config.operation_timeout > 0.0:
            self.recognizer.operation_timeout = config.operation_timeout
        else:
            self.recognizer.operation_timeout = None

        # config for VAD
        if config.pause_threshold < config.non_speaking_duration:
            config.pause_threshold = config.non_speaking_duration
        self.recognizer.pause_threshold = config.pause_threshold
        self.recognizer.non_speaking_duration = config.non_speaking_duration
        self.recognizer.phrase_threshold = config.phrase_threshold

        return config

    def play_sound(self, key, timeout=5.0):
        if self.act_sound is None:
            return
        req = SoundRequest()
        req.sound = SoundRequest.PLAY_FILE
        req.command = SoundRequest.PLAY_ONCE
        if hasattr(SoundRequest, 'volume'): # volume is added from 0.3.0 https://github.com/ros-drivers/audio_common/commit/da9623414f381642e52f59701c09928c72a54be7#diff-fe2d85580f1ccfed4e23a608df44a7f7
            req.volume = 1.0
        req.arg = self.signals[key]
        goal = SoundRequestGoal(sound_request=req)
        self.act_sound.send_goal_and_wait(goal, timeout)
        # ros_compat's send_goal_and_wait() returns whether the goal finished,
        # not its state, and ROS2's client has no get_state()
        if hasattr(self.act_sound, 'get_state') and \
           self.act_sound.get_state() == GoalStatus.ABORTED:
            ros_compat.logwarn(self.node,
                "Playing %s signal was aborted. "
                "Does its sound file exist on the machine "
                "on which sound_play is running?"
                % key
            )

    def recognize(self, audio):
        recog_func = None
        if self.engine == Config.SpeechRecognition_Google:
            if not self.args:
                self.args = {'key': ros_compat.get_param(self.node, "~google_key", None),
                             'show_all': True}
            recog_func = self.recognizer.recognize_google
        elif self.engine == Config.SpeechRecognition_GoogleCloud:
            if not self.args:
                credentials_path = ros_compat.get_param(self.node, "~google_cloud_credentials_json", None)
                if credentials_path is not None:
                    with open(credentials_path) as j:
                        credentials_json = j.read()
                else:
                    credentials_json = None
                self.args = {'credentials_json': credentials_json,
                             'preferred_phrases': ros_compat.get_param(self.node, '~google_cloud_preferred_phrases', None)}
                if ros_compat.has_param(self.node, '~diarizationConfig') :
                    self.args.update({'user_config': {'diarizationConfig': ros_compat.get_param(self.node, '~diarizationConfig') }})
            recog_func = self.recognizer.recognize_google_cloud
        elif self.engine == Config.SpeechRecognition_Sphinx:
            recog_func = self.recognizer.recognize_sphinx
        elif self.engine == Config.SpeechRecognition_Wit:
            recog_func = self.recognizer.recognize_wit
        elif self.engine == Config.SpeechRecognition_Bing:
            if not self.args:
                self.args = {'key': ros_compat.get_param(self.node, "~bing_key")}
            recog_func = self.recognizer.recognize_bing
        elif self.engine == Config.SpeechRecognition_Houndify:
            recog_func = self.recognizer.recognize_houndify
        elif self.engine == Config.SpeechRecognition_IBM:
            recog_func = self.recognizer.recognize_ibm
        elif self.engine == Config.SpeechRecognition_Vosk:
            if not self.args:
                self.args = {'model_path': ros_compat.get_param(self.node, '~vosk_model_path', None)}
            recog_func = self.recognizer.recognize_vosk

        return recog_func(audio_data=audio, language=self.language, **self.args)

    def audio_cb(self, _, audio):
        if not self.enable_audio_cb:
            return
        if self.is_canceling:
            ros_compat.loginfo(self.node, "Robot is speaking now, so recognition is cancelled")
            return
        try:
            ros_compat.logdebug(self.node, "Waiting for result... (Sent %d bytes)" % len(audio.get_raw_data()))
            result = self.recognize(audio)
            confidence = 1.0
            if len(result) == 0: return
            if self.engine == Config.SpeechRecognition_Google:
                confidence = result['alternative'][0]['confidence']
                result = result['alternative'][0]['transcript']
            self.play_sound("recognized", 0.05)
            ros_compat.loginfo(self.node, "Result: %s" % result.encode('utf-8'))
            self.play_sound("success", 0.1)
            msg = SpeechRecognitionCandidates(
                transcript=[result],
                confidence=[confidence],
            )
            self.pub.publish(msg)
            return
        except SR.UnknownValueError as e:
            if self.dynamic_energy_threshold:
                self.recognizer.adjust_for_ambient_noise(self.audio)
                ros_compat.loginfo(self.node, "Updated energy threshold to %f" % self.recognizer.energy_threshold)
        except SR.RequestError as e:
            ros_compat.logerr(self.node, "Failed to recognize: %s" % str(e))
        except SR.UnknownValueError as e:
            ros_compat.logerr(self.node, "Failed to recognize: %s" % str(e))
        except:
            ros_compat.logerr(self.node, "Unexpected error: %s" % str(sys.exc_info()))
        self.play_sound("timeout", 0.1)

    def start_speech_recognition(self):
        if self.engine == Config.SpeechRecognition_GoogleCloudStream:
            self.play_sound("start", 0.1)
            self._stream_stop_evt = Event()
            self._stream_thread = Thread(target=self._streaming_loop, daemon=True)
            self._stream_thread.start()
            rospy.on_shutdown(self.on_shutdown)
            return
        if self.dynamic_energy_threshold:
            with self.audio as src:
                self.recognizer.adjust_for_ambient_noise(src)
                ros_compat.loginfo(self.node, "Set minimum energy threshold to {}".format(
                    self.recognizer.energy_threshold))
        self.play_sound("start", 0.1)
        self.stop_fn = self.recognizer.listen_in_background(
            self.audio, self.audio_cb, phrase_time_limit=self.phrase_time_limit)
        ros_compat.on_shutdown(self.node, self.on_shutdown)

    def stop_speech_recognition(self):
        if self._stream_stop_evt is not None:
            self._stream_stop_evt.set()
            if self._stream_thread is not None:
                self._stream_thread.join(timeout=2.0)
            self._stream_thread = None
            self._stream_stop_evt = None
            if self._stream_recognizer is not None:
                self._stream_recognizer.stop_stream()
        if self.stop_fn is not None:
            self.stop_fn()

    def _streaming_loop(self):
        sr = self.audio.SAMPLE_RATE
        chunk_bytes = sr * 2 * 100 // 1000  # 100 ms of S16 mono
        was_canceling = False
        last_logged_interim = ""
        with self.audio as src:
            while (not rospy.is_shutdown()
                   and not self._stream_stop_evt.is_set()
                   and self.enable_audio_cb):
                chunk = src.stream.read(chunk_bytes)
                if not chunk:
                    continue
                # Tear the gRPC stream down while the robot is speaking;
                # accept_waveform() lazily reopens it once is_canceling clears.
                if self.is_canceling:
                    if not was_canceling:
                        self._stream_recognizer.stop_stream()
                        was_canceling = True
                    continue
                was_canceling = False
                arr = np.frombuffer(chunk, dtype=np.int16).astype(np.float32) / 32768.0
                try:
                    self._stream_recognizer.accept_waveform(sr, arr)
                except Exception as e:
                    rospy.logerr("streaming accept_waveform failed: %s", e)
                    continue
                if self._stream_recognizer.is_endpoint(None):
                    text = self._stream_recognizer.get_result(None)
                    self._stream_recognizer.reset(None)
                    if text and text.strip():
                        self.play_sound("recognized", 0.05)
                        rospy.loginfo("Result: %s", text)
                        self.pub.publish(SpeechRecognitionCandidates(
                            transcript=[text], confidence=[1.0]))
                        self.play_sound("success", 0.1)
                    last_logged_interim = ""
                else:
                    interim = self._stream_recognizer.get_result(None)
                    if interim:
                        if self.pub_interim is not None:
                            self.pub_interim.publish(String(data=interim))
                        if self._log_interim and interim != last_logged_interim:
                            rospy.loginfo("Interim: %s", interim)
                            last_logged_interim = interim

    def on_shutdown(self):
        self.stop_speech_recognition()

    def speech_recognition_srv_cb(self, req, res):
        duration = req.duration
        if duration <= 0.0:
            duration = self.default_duration

        with self.audio as src:
            if self.dynamic_energy_threshold:
                self.recognizer.adjust_for_ambient_noise(src)
                ros_compat.loginfo(self.node, "Set minimum energy threshold to %f" % self.recognizer.energy_threshold)

            if not req.quiet:
                self.play_sound("start", 0.1)

            start_time = ros_compat.now_sec(self.node)
            while ros_compat.now_sec(self.node) - start_time < duration:
                ros_compat.loginfo(self.node, "Waiting for speech...")
                try:
                    audio = self.recognizer.listen(
                        src, timeout=self.listen_timeout, phrase_time_limit=self.phrase_time_limit)
                except SR.WaitTimeoutError as e:
                    ros_compat.logwarn(self.node, str(e))
                    break
                if not req.quiet:
                    self.play_sound("recognized", 0.05)
                ros_compat.loginfo(self.node, "Waiting for result... (Sent %d bytes)" % len(audio.get_raw_data()))

                try:
                    result = self.recognize(audio)
                    confidence = 1.0
                    # recognize_google() returns all the alternatives
                    # (show_all=True), so take the best one as audio_cb()
                    # does; this failed before (from Michi-Tsubaki's ROS2 port)
                    if self.engine == Config.SpeechRecognition_Google:
                        if len(result) == 0:
                            raise SR.UnknownValueError()
                        confidence = result['alternative'][0].get('confidence', 1.0)
                        result = result['alternative'][0]['transcript']
                    ros_compat.loginfo(self.node, "Result: %s" % result.encode('utf-8'))
                    if not req.quiet:
                        self.play_sound("success", 0.1)
                    res.result = SpeechRecognitionCandidates(
                        transcript=[result],
                        confidence=[confidence],
                    )
                    return res
                except SR.UnknownValueError:
                    if self.dynamic_energy_threshold:
                        self.recognizer.adjust_for_ambient_noise(src)
                        ros_compat.loginfo(self.node, "Set minimum energy threshold to %f" % self.recognizer.energy_threshold)
                except SR.RequestError as e:
                    ros_compat.logerr(self.node, "Failed to recognize: %s" % str(e))
                ros_compat.sleep(self.node, 0.1)
                if ros_compat.is_shutdown(self.node):
                    break

            # Timeout
            if not req.quiet:
                self.play_sound("timeout", 0.1)
            return res

    def speech_recogniton_start_srv_cb(self, req, res):
        self.enable_audio_cb = True
        self.start_speech_recognition()
        ros_compat.loginfo(self.node, "Start continuous mode")
        return res

    def speech_recogniton_stop_srv_cb(self, req, res):
        self.enable_audio_cb = False
        self.stop_speech_recognition()
        ros_compat.loginfo(self.node, "Stop continuous mode")
        return res

    def spin(self):
        if self.continuous and self.auto_start:
            # in a thread, as it waits for the audio topic, which is received
            # only while spinning on ROS2 (Michi-Tsubaki's ROS2 port starts it
            # from a timer for the same reason)
            start_thread = Thread(target=self.start_speech_recognition)
            start_thread.daemon = True
            start_thread.start()
        ros_compat.spin(self.node)

    def tts_timer_cb(self, event):
        stamp = ros_compat.now_sec(self.node)
        active = False
        for tts_action in self.tts_actions:
            status = self.tts_status.get(tts_action)
            for st in (status.status_list if status else []):
                if st.status == TTS_STATUS_ACTIVE:
                    active = True
                    break
            if active:
                break
        if active:
            if not self.is_canceling:
                ros_compat.logdebug(self.node, "START CANCELLATION")
                self.is_canceling = True
                self.last_tts = None
        elif self.is_canceling:
            if self.last_tts is None:
                self.last_tts = stamp
            if stamp - self.last_tts > self.tts_tolerance:
                ros_compat.logdebug(self.node, "END CANCELLATION")
                self.is_canceling = False


if __name__ == '__main__':
    # the recognition service callback waits for the audio topic and the
    # sound_play action (on ROS2, a MultiThreadedExecutor and reentrant
    # callbacks, as in Michi-Tsubaki's ROS2 port of this node)
    node = ros_compat.init_node("speech_recognition", multi_threaded=True)
    rec = ROSSpeechRecognition(node)
    rec.spin()
