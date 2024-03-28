#!/usr/bin/env python

import sys

import rospy
from audio_common_msgs.msg import AudioData, AudioInfo
from std_msgs.msg import Bool


class VADBaseNode(object):

    def __init__(self, chunk_size: int = 480):

        self._last_speaking_time = rospy.Time()
        self._speech_audio_buffer = b""
        self._audio_buffer = b""

        self._threshold = rospy.get_param("~threshold", 0.1)
        self._minimum_duration = rospy.get_param("~minimum_duration", 1.0)
        self._maximum_duration = rospy.get_param("~maximum_duration", 10.0)
        self._audio_timeout_duration = rospy.get_param("~audio_timeout_duration", 0.3)

        self._pub_is_speech = rospy.Publisher("~is_speeching", Bool, queue_size=1)
        self._pub_speech_audio = rospy.Publisher(
            "~speech_audio", AudioData, queue_size=1
        )
        self._pub_speech_audio_info = rospy.Publisher(
            "~speech_audio_info", AudioInfo, queue_size=1, latch=True
        )

        self._audio_info = rospy.wait_for_message("audio_info", AudioInfo)
        if self._audio_info.sample_format != "S16LE":
            rospy.logerr("audio format must be S16LE")
            sys.exit(1)
        if self._audio_info.sample_rate not in [8000, 16000, 32000, 48000]:
            rospy.logerr("sampling rate must be 8000 or 16000 or 32000 or 48000")
            sys.exit(1)

        self.chunk_size = chunk_size
        self._pub_speech_audio_info.publish(self._audio_info)
        self._sub = rospy.Subscriber("audio_data", AudioData, self._callback)

    def _get_vad_confidence(self, chunk: bytearray, sampling_rate: int) -> float:
        raise NotImplementedError

    def _callback(self, msg):
        self._audio_buffer += msg.data
        if len(self._audio_buffer) < 2 * self.chunk_size:
            return
        else:
            audio_data = self._audio_buffer[: 2 * self.chunk_size]
            self._audio_buffer = self._audio_buffer[2 * self.chunk_size :]

        confidence = self._get_vad_confidence(audio_data, self._audio_info.sample_rate)
        rospy.logdebug("confidence: {}".format(confidence))
        is_speech = True if confidence > self._threshold else False
        self._pub_is_speech.publish(Bool(is_speech))

        if not is_speech and len(self._speech_audio_buffer) == 0:
            return

        self._speech_audio_buffer = self._speech_audio_buffer + audio_data
        speech_duration: float = (
            (len(self._speech_audio_buffer) / 2.0) / self._audio_info.sample_rate
            - self.chunk_size / self._audio_info.sample_rate
            - self._audio_timeout_duration
        )
        if is_speech and speech_duration < self._maximum_duration:
            self._last_speaking_time = rospy.Time.now()
        else:
            if rospy.Time.now() < self._last_speaking_time + rospy.Duration(
                self._audio_timeout_duration
            ):
                rospy.logdebug("continuing...")
            else:
                if speech_duration > self._minimum_duration:
                    self._pub_speech_audio.publish(
                        AudioData(self._speech_audio_buffer)
                    )
                    rospy.loginfo("published duration: {}".format(speech_duration))
                else:
                    rospy.logwarn(
                        "speech duration: {} dropped".format(speech_duration)
                    )
                self._speech_audio_buffer = b''
