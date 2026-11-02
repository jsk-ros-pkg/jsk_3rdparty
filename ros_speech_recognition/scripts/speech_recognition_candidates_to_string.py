#!/usr/bin/env python

from ros_speech_recognition.compat import ros_compat
from ros_speech_recognition.compat import ROS_VERSION

if ROS_VERSION == 1:
    from jsk_topic_tools import ConnectionBasedTransport
else:  # no ROS2 python ConnectionBasedTransport in jsk_topic_tools, see __main__
    ConnectionBasedTransport = object

from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from std_msgs.msg import String


class SpeechRecognitionCandidatesToString(ConnectionBasedTransport):
    def __init__(self, node=None):
        self.node = node
        super(SpeechRecognitionCandidatesToString, self).__init__()
        self.pub = self.advertise('~output', String, queue_size=1)

    def subscribe(self):
        self.sub = ros_compat.create_subscription(self.node,
            '~input', SpeechRecognitionCandidates, self._cb)

    def unsubscribe(self):
        self.sub.unregister()

    def _cb(self, can_msg):
        str_msg = String()
        str_msg.data = can_msg.transcript[0]
        self.pub.publish(str_msg)


if __name__ == '__main__':
    node = ros_compat.init_node('speech_recognition_candidates_to_string')
    if ROS_VERSION == 1:
        app = SpeechRecognitionCandidatesToString(node)
    else:
        # always subscribe, without ConnectionBasedTransport
        pub = ros_compat.create_publisher(node, '~output', String, queue_size=1)
        sub = ros_compat.create_subscription(node,
            '~input', SpeechRecognitionCandidates,
            lambda msg: pub.publish(String(data=msg.transcript[0])))
    ros_compat.spin(node)
