#!/usr/bin/env python

import webrtcvad
import rospy
from audio_common_msgs.msg import AudioData, AudioInfo
from std_msgs.msg import Bool


class WebRTCVADROS(object):

    def __init__(self):

        self._current_speaking = False
        self._speech_audio_buffer = b''

        self._queue_duration = rospy.get_param('~queue_duration', 30)  # ms
        aggressiveness = rospy.get_param('~aggressiveness', 1)
        self._minimum_duration = rospy.get_param('~minimum_duration', 0.4)
        self._vad = webrtcvad.Vad(int(aggressiveness))

        self._pub_is_speech = rospy.Publisher(
            '~is_speeching', Bool, queue_size=1)
        self._pub_speech_audio = rospy.Publisher(
            '~speech_audio', AudioData, queue_size=1)
        self._pub_speech_audio_info = rospy.Publisher(
            '~speech_audio_info', AudioInfo, queue_size=1, latch=True)

        self._audio_info = rospy.wait_for_message('audio_info', AudioInfo)
        if self._audio_info.sample_format != 'S16LE':
            rospy.logerr('audio format must be S16LE')
            return
        if self._audio_info.sample_rate not in [8000, 16000, 32000, 48000]:
            rospy.logerr(
                'sampling rate must be 8000 or 16000 or 32000 or 48000')
            return

        self.length_queue_popup = int(
            2 * (self._audio_info.sample_rate * self._queue_duration)/1000)
        self.audio_data_buffer = b''

        self._pub_speech_audio_info.publish(self._audio_info)
        self._sub = rospy.Subscriber('audio_data', AudioData, self._callback)

    def _callback(self, msg):
        # append buffer and pop data with specified length
        self.audio_data_buffer += msg.data
        while len(self.audio_data_buffer) > self.length_queue_popup:
            self.publish()

    def publish(self):
        # Input Data
        input_data = self.audio_data_buffer[:self.length_queue_popup]
        self.audio_data_buffer = self.audio_data_buffer[self.length_queue_popup:]
        try:
            is_speech = self._vad.is_speech(
                input_data, self._audio_info.sample_rate)
        except Exception as e:
            rospy.logerr('Got an error while processing. {}'.format(e))
            return
        self._pub_is_speech.publish(Bool(is_speech))
        if self._current_speaking == True and is_speech == True:
            self._speech_audio_buffer = self._speech_audio_buffer + input_data
        elif self._current_speaking == False and is_speech == True:
            self._speech_audio_buffer = input_data
            self._current_speaking = True
        elif self._current_speaking == True and is_speech == False:
            self._speech_audio_buffer = self._speech_audio_buffer + input_data
            speech_duration = (len(self._speech_audio_buffer) /
                               2.0) / self._audio_info.sample_rate
            if speech_duration > self._minimum_duration:
                self._pub_speech_audio.publish(
                    AudioData(self._speech_audio_buffer))
            else:
                rospy.logwarn(
                    'speech duration: {} dropped'.format(speech_duration))
            self._current_speaking = False
            self._speech_audio_buffer = b''


def main():

    rospy.init_node('webrtcvad_ros')
    node = WebRTCVADROS()
    rospy.spin()


if __name__ == '__main__':
    main()
