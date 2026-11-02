#!/usr/bin/env python

from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat

from jsk_topic_tools import ConnectionBasedTransport

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
    app = SpeechRecognitionCandidatesToString(node)
    ros_compat.spin(node)
