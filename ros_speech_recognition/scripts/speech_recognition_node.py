#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import actionlib
import rospy
import speech_recognition as SR
from ros_speech_recognition.recognize_google_cloud import RecognizerEx
import json
import array
import sys
from threading import Lock

from audio_common_msgs.msg import AudioData
from sound_play.msg import SoundRequest, SoundRequestAction, SoundRequestGoal

from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from speech_recognition_msgs.srv import SpeechRecognition
from speech_recognition_msgs.srv import SpeechRecognitionResponse

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
        self.act_sound = actionlib.SimpleActionClient("sound_play", SoundRequestAction)
        if not self.act_sound.wait_for_server(rospy.Duration(5.0)):
            rospy.logwarn("Failed to find sound_play action. Disabled audio alert")
            self.act_sound = None
        self.signals = {
            "start": rospy.get_param("~start_signal",
                                     "/usr/share/sounds/ubuntu/stereo/bell.ogg"),
            "recognized": rospy.get_param("~recognized_signal",
                                          "/usr/share/sounds/ubuntu/stereo/button-toggle-on.ogg"),
            "success": rospy.get_param("~success_signal",
                                       "/usr/share/sounds/ubuntu/stereo/message-new-instant.ogg"),
            "timeout": rospy.get_param("~timeout_signal",
                                       "/usr/share/sounds/ubuntu/stereo/window-slide.ogg"),
        }

        self.dyn_srv = Server(Config, self.config_callback)

        self.stop_fn = None
        self.continuous = rospy.get_param("~continuous", False)
        if self.continuous:
            rospy.loginfo("Enabled continuous mode")
            self.pub = rospy.Publisher(rospy.get_param("~voice_topic", "/Tablet/voice"),
                                       SpeechRecognitionCandidates,
                                       queue_size=1)
        else:
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
                self.args = {'key': rospy.get_param("~google_key", None)}
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

        return recog_func(audio_data=audio, language=self.language, **self.args)

    def audio_cb(self, _, audio):
        try:
            rospy.logdebug("Waiting for result... (Sent %d bytes)" % len(audio.get_raw_data()))
            result = self.recognize(audio)
            rospy.loginfo("Result: %s" % result.encode('utf-8'))
            msg = SpeechRecognitionCandidates(transcript=[result])
            self.pub.publish(msg)
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

    def start_speech_recognition(self):
        if self.dynamic_energy_threshold:
            with self.audio as src:
                self.recognizer.adjust_for_ambient_noise(src)
                rospy.loginfo("Set minimum energy threshold to {}".format(
                    self.recognizer.energy_threshold))
        self.stop_fn = self.recognizer.listen_in_background(
            self.audio, self.audio_cb, phrase_time_limit=self.phrase_time_limit)
        rospy.on_shutdown(self.on_shutdown)

    def on_shutdown(self):
        if self.stop_fn is not None:
            self.stop_fn()

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
                    res.result = SpeechRecognitionCandidates(transcript=[result])
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

    def spin(self):
        if self.continuous:
            self.start_speech_recognition()
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node("speech_recognition")
    rec = ROSSpeechRecognition()
    rec.spin()
