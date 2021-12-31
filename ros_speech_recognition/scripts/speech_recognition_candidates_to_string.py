#!/usr/bin/env python

import rospy

from jsk_topic_tools import ConnectionBasedTransport

from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from std_msgs.msg import String


class SpeechRecognitionCandidatesToString(ConnectionBasedTransport):
    def __init__(self):
        super(SpeechRecognitionCandidatesToString, self).__init__()
        self.pub = self.advertise('~output', String, queue_size=1)

    def subscribe(self):
        self.sub = rospy.Subscriber(
            '~input', SpeechRecognitionCandidates, self._cb)

    def unsubscribe(self):
        self.sub.unregister()

    def _cb(self, can_msg):
        str_msg = String()
        str_msg.data = can_msg.transcript[0]
        self.pub.publish(str_msg)


if __name__ == '__main__':
    rospy.init_node('speech_recognition_candidates_to_string')
    app = SpeechRecognitionCandidatesToString()
    rospy.spin()
