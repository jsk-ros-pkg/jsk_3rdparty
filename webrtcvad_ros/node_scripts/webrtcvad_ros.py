#!/usr/bin/env python

import webrtcvad
import rospy
from audio_common_msgs.msg import AudioData, AudioInfo
from std_msgs.msg import Bool

class WebRTCVADROS(object):

    def __init__(self):

        self._current_speaking = False
        self._speech_audio_buffer = ''

        aggressiveness = rospy.get_param('~aggressiveness',1)
        self._minimum_duration = rospy.get_param('~minimum_duration',0.4)
        self._vad = webrtcvad.Vad(int(aggressiveness))

        self._pub_is_speech = rospy.Publisher('~is_speeching',Bool,queue_size=1)
        self._pub_speech_audio = rospy.Publisher('~speech_audio',AudioData,queue_size=1)
        self._pub_speech_audio_info = rospy.Publisher('~speech_audio_info',AudioInfo,queue_size=1,latch=True)

        self._audio_info = rospy.wait_for_message('audio_info',AudioInfo)
        if self._audio_info.sample_format != 'S16LE':
            rospy.logerr('audio format must be S16LE')
            return
        if self._audio_info.sample_rate not in [8000, 16000, 32000, 48000]:
            rospy.logerr('sampling rate must be 8000 or 16000 or 32000 or 48000')
            return

        self._pub_speech_audio_info.publish(self._audio_info)
        self._sub = rospy.Subscriber('audio_data',AudioData,self._callback)

    def _callback(self, msg):
        is_speech = self._vad.is_speech(msg.data, self._audio_info.sample_rate)
        self._pub_is_speech.publish(Bool(is_speech))
        if self._current_speaking == True and is_speech == True:
            self._speech_audio_buffer = self._speech_audio_buffer + msg.data
        elif self._current_speaking == False and is_speech == True:
            self._speech_audio_buffer = msg.data
            self._current_speaking = True
        elif self._current_speaking == True and is_speech == False:
            self._speech_audio_buffer = self._speech_audio_buffer + msg.data
            speech_duration = (len(self._speech_audio_buffer) / 2.0) / self._audio_info.sample_rate
            if speech_duration > self._minimum_duration: 
                self._pub_speech_audio.publish(AudioData(self._speech_audio_buffer))
            else:
                rospy.logwarn('speech duration: {} dropped'.format(speech_duration))
            self._current_speaking = False
            self._speech_audio_buffer = ''

def main():

    rospy.init_node('webrtcvad_ros')
    node = WebRTCVADROS()
    rospy.spin()

if __name__=='__main__':
    main()
