#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import actionlib
import rospy
import speech_recognition as SR
from ros_speech_recognition.recognize_google_cloud import RecognizerEx
import ros_speech_recognition.recognize_vosk
import json
import array
import sys
from threading import Lock

from audio_common_msgs.msg import AudioData
from sound_play.msg import SoundRequest, SoundRequestAction, SoundRequestGoal

from actionlib_msgs.msg import GoalStatus
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from speech_recognition_msgs.srv import SpeechRecognition
from speech_recognition_msgs.srv import SpeechRecognitionResponse
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse

from dynamic_reconfigure.server import Server
from ros_speech_recognition.cfg import SpeechRecognitionConfig as Config


class ROSAudio(SR.AudioSource):
    def __init__(self, topic_name="audio", depth=16, n_channel=1,
                 sample_rate=16000, chunk_size=1024, buffer_size=10240):
        assert buffer_size > chunk_size

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
            n_channel=self.n_channel)
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
                     n_channel=1, target_channel=0):
            self.buffer_size = buffer_size
            self.lock = Lock()
            self.buffer = bytes()
            self.depth = depth
            self.n_channel = n_channel
            self.target_channel = min(self.n_channel - 1, max(0, target_channel))
            self.sub_audio = rospy.Subscriber(
                topic_name, AudioData, self.audio_cb)
            self.type_code = {}
            for code in ['b', 'h', 'i', 'l']:
                self.type_code[array.array(code).itemsize] = code

        def read_once(self, size):
            with self.lock:
                buf = self.buffer[:size]
                self.buffer = self.buffer[size:]
                return buf

        def read(self, size):
            while not rospy.is_shutdown() and len(self.buffer) < size:
                rospy.sleep(0.001)
            return self.read_once(size)

        def close(self):
            try:
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
                self.buffer += array.array(dtype, chan_data).tostring()
                overflow = len(self.buffer) - self.buffer_size
                if overflow > 0:
                    self.buffer = self.buffer[overflow:]


class ROSSpeechRecognition(object):
    def __init__(self):
        self.default_duration = rospy.get_param("~duration", 10.0)
        self.engine = None
        self.recognizer = RecognizerEx() # Use custom Recognizer to support exteded API
        self.audio = ROSAudio(topic_name=rospy.get_param("~audio_topic", "audio"),
                              depth=rospy.get_param("~depth", 16),
                              n_channel=rospy.get_param("~n_channel", 1),
                              sample_rate=rospy.get_param("~sample_rate", 16000),
                              buffer_size=rospy.get_param("~buffer_size", 10240))

        # initialize sound play client
        if rospy.get_param('~enable_sound_effect', True):
            self.act_sound = actionlib.SimpleActionClient(
                "sound_play", SoundRequestAction)
            if not self.act_sound.wait_for_server(rospy.Duration(5.0)):
                rospy.logwarn("Failed to find sound_play action. "
                              "Disabled sound effect on recognition.")
                self.act_sound = None
        else:
            self.act_sound = None
        self.signals = {
            "start": rospy.get_param("~start_signal",
                                     "/usr/share/sounds/freedesktop/stereo/bell.ogg"),
            "recognized": rospy.get_param("~recognized_signal",
                                          "/usr/share/sounds/freedesktop/stereo/message.ogg"),
            "success": rospy.get_param("~success_signal",
                                       "/usr/share/sounds/freedesktop/stereo/message-new-instant.ogg"),
            "timeout": rospy.get_param("~timeout_signal",
                                       "/usr/share/sounds/freedesktop/stereo/network-connectivity-lost.ogg"),
        }


        # ignore voice input while the robot is speaking
        self.self_cancellation = rospy.get_param("~self_cancellation", True)
        # time to assume as SPEAKING after tts service is finished
        self.tts_tolerance = rospy.Duration.from_sec(
            rospy.get_param("~tts_tolerance", 1.0))
        tts_action_names = rospy.get_param(
            '~tts_action_names', ['sound_play'])

        self.tts_action = None
        self.last_tts = None
        self.is_canceling = False
        self.tts_actions = []
        if self.self_cancellation:
            for tts_action_name in tts_action_names:
                tts_action = actionlib.SimpleActionClient(
                    tts_action_name, SoundRequestAction)
                if tts_action.wait_for_server(rospy.Duration(5.0)):
                    self.tts_actions.append(tts_action)
                else:
                    rospy.logerr(
                        "action '{}' is not initialized."
                        .format(tts_action_name))
                self.tts_timer = rospy.Timer(rospy.Duration(0.1), self.tts_timer_cb)

        self.dyn_srv = Server(Config, self.config_callback)

        self.stop_fn = None
        self.continuous = rospy.get_param("~continuous", False)
        self.auto_start = rospy.get_param("~auto_start", True)
        self.enable_audio_cb = self.auto_start
        if self.continuous:
            rospy.loginfo("Enabled continuous mode")
            rospy.loginfo("Auto start: {}".format(self.auto_start))
            self.pub = rospy.Publisher(rospy.get_param("~voice_topic", "speech_to_text"),
                                       SpeechRecognitionCandidates,
                                       queue_size=1)
            self.start_srv = rospy.Service(
                "speech_recognition/start",
                Empty, self.speech_recogniton_start_srv_cb)
            self.stop_srv = rospy.Service(
                "speech_recognition/stop",
                Empty, self.speech_recogniton_stop_srv_cb)
        else:
            rospy.loginfo("Disabled continuous mode, waiting for speech_recognition service")
            self.srv = rospy.Service("speech_recognition",
                                     SpeechRecognition,
                                     self.speech_recognition_srv_cb)

    def config_callback(self, config, level):
        # config for engine
        self.language = config.language
        if self.engine != config.engine:
            self.args = {}
            self.engine = config.engine

        # config for adaptive thresholding
        self.dynamic_energy_threshold = config.dynamic_energy_threshold
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
        self.act_sound.send_goal_and_wait(goal, rospy.Duration(timeout))

    def recognize(self, audio):
        recog_func = None
        if self.engine == Config.SpeechRecognition_Google:
            if not self.args:
                self.args = {'key': rospy.get_param("~google_key", None),
                             'show_all': True}
            recog_func = self.recognizer.recognize_google
        elif self.engine == Config.SpeechRecognition_GoogleCloud:
            if not self.args:
                credentials_path = rospy.get_param("~google_cloud_credentials_json", None)
                if credentials_path is not None:
                    with open(credentials_path) as j:
                        credentials_json = j.read()
                else:
                    credentials_json = None
                self.args = {'credentials_json': credentials_json,
                             'preferred_phrases': rospy.get_param('~google_cloud_preferred_phrases', None)}
                if rospy.has_param('~diarizationConfig') :
                    self.args.update({'user_config': {'diarizationConfig': rospy.get_param('~diarizationConfig') }})
            recog_func = self.recognizer.recognize_google_cloud
        elif self.engine == Config.SpeechRecognition_Sphinx:
            recog_func = self.recognizer.recognize_sphinx
        elif self.engine == Config.SpeechRecognition_Wit:
            recog_func = self.recognizer.recognize_wit
        elif self.engine == Config.SpeechRecognition_Bing:
            if not self.args:
                self.args = {'key': rospy.get_param("~bing_key")}
            recog_func = self.recognizer.recognize_bing
        elif self.engine == Config.SpeechRecognition_Houndify:
            recog_func = self.recognizer.recognize_houndify
        elif self.engine == Config.SpeechRecognition_IBM:
            recog_func = self.recognizer.recognize_ibm
        elif self.engine == Config.SpeechRecognition_Vosk:
            if not self.args:
                self.args = {'model_path': rospy.get_param('~vosk_model_path', None)}
            recog_func = self.recognizer.recognize_vosk

        return recog_func(audio_data=audio, language=self.language, **self.args)

    def audio_cb(self, _, audio):
        if not self.enable_audio_cb:
            return
        if self.is_canceling:
            rospy.loginfo("Robot is speaking now, so recognition is cancelled")
            return
        try:
            rospy.logdebug("Waiting for result... (Sent %d bytes)" % len(audio.get_raw_data()))
            result = self.recognize(audio)
            confidence = 1.0
            if len(result) == 0: return
            if self.engine == Config.SpeechRecognition_Google:
                confidence = result['alternative'][0]['confidence']
                result = result['alternative'][0]['transcript']
            self.play_sound("recognized", 0.05)
            rospy.loginfo("Result: %s" % result.encode('utf-8'))
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
                rospy.loginfo("Updated energy threshold to %f" % self.recognizer.energy_threshold)
        except SR.RequestError as e:
            rospy.logerr("Failed to recognize: %s" % str(e))
        except SR.UnknownValueError as e:
            rospy.logerr("Failed to recognize: %s" % str(e))
        except:
            rospy.logerr("Unexpected error: %s" % str(sys.exc_info()))
        self.play_sound("timeout", 0.1)

    def start_speech_recognition(self):
        if self.dynamic_energy_threshold:
            with self.audio as src:
                self.recognizer.adjust_for_ambient_noise(src)
                rospy.loginfo("Set minimum energy threshold to {}".format(
                    self.recognizer.energy_threshold))
        self.play_sound("start", 0.1)
        self.stop_fn = self.recognizer.listen_in_background(
            self.audio, self.audio_cb, phrase_time_limit=self.phrase_time_limit)
        rospy.on_shutdown(self.on_shutdown)

    def stop_speech_recognition(self):
        if self.stop_fn is not None:
            self.stop_fn()

    def on_shutdown(self):
        self.stop_speech_recognition()

    def speech_recognition_srv_cb(self, req):
        res = SpeechRecognitionResponse()

        duration = req.duration
        if duration <= 0.0:
            duration = self.default_duration

        with self.audio as src:
            if self.dynamic_energy_threshold:
                self.recognizer.adjust_for_ambient_noise(src)
                rospy.loginfo("Set minimum energy threshold to %f" % self.recognizer.energy_threshold)

            if not req.quiet:
                self.play_sound("start", 0.1)

            start_time = rospy.Time.now()
            while (rospy.Time.now() - start_time).to_sec() < duration:
                rospy.loginfo("Waiting for speech...")
                try:
                    audio = self.recognizer.listen(
                        src, timeout=self.listen_timeout, phrase_time_limit=self.phrase_time_limit)
                except SR.WaitTimeoutError as e:
                    rospy.logwarn(e)
                    break
                if not req.quiet:
                    self.play_sound("recognized", 0.05)
                rospy.loginfo("Waiting for result... (Sent %d bytes)" % len(audio.get_raw_data()))

                try:
                    result = self.recognize(audio)
                    rospy.loginfo("Result: %s" % result.encode('utf-8'))
                    if not req.quiet:
                        self.play_sound("success", 0.1)
                    res.result = SpeechRecognitionCandidates(
                        transcript=[result],
                        confidence=[1.0],
                    )
                    return res
                except SR.UnknownValueError:
                    if self.dynamic_energy_threshold:
                        self.recognizer.adjust_for_ambient_noise(src)
                        rospy.loginfo("Set minimum energy threshold to %f" % self.recognizer.energy_threshold)
                except SR.RequestError as e:
                    rospy.logerr("Failed to recognize: %s" % str(e))
                rospy.sleep(0.1)
                if rospy.is_shutdown():
                    break

            # Timeout
            if not req.quiet:
                self.play_sound("timeout", 0.1)
            return res

    def speech_recogniton_start_srv_cb(self, req):
        self.enable_audio_cb = True
        self.start_speech_recognition()
        rospy.loginfo("Start continuous mode")
        return EmptyResponse()

    def speech_recogniton_stop_srv_cb(self, req):
        self.enable_audio_cb = False
        self.stop_speech_recognition()
        rospy.loginfo("Stop continuous mode")
        return EmptyResponse()

    def spin(self):
        if self.continuous and self.auto_start:
            self.start_speech_recognition()
        rospy.spin()

    def tts_timer_cb(self, event):
        stamp = event.current_real
        active = False
        for tts_action in self.tts_actions:
            for st in tts_action.action_client.last_status_msg.status_list:
                if st.status == GoalStatus.ACTIVE:
                    active = True
                    break
            if active:
                break
        if active:
            if not self.is_canceling:
                rospy.logdebug("START CANCELLATION")
                self.is_canceling = True
                self.last_tts = None
        elif self.is_canceling:
            if self.last_tts is None:
                self.last_tts = stamp
            if stamp - self.last_tts > self.tts_tolerance:
                rospy.logdebug("END CANCELLATION")
                self.is_canceling = False


if __name__ == '__main__':
    rospy.init_node("speech_recognition")
    rec = ROSSpeechRecognition()
    rec.spin()
