#!/usr/bin/env python

from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat
from sound_play.libsoundplay import SoundClient

from speech_recognition_msgs.msg import SpeechRecognitionCandidates


class ParrotNode(object):
    def __init__(self, node=None):
        self.sub = ros_compat.create_subscription(node,
            '~input',
            SpeechRecognitionCandidates,
            self._sub_cb,
            queue_size=1
        )
        tts_action_name = ros_compat.get_param(node,
            '~tts_action_name', 'sound_play')
        self.sound_client = SoundClient(
            blocking=True,
            sound_action=tts_action_name,
        )
        self.conf_thresh = ros_compat.get_param(node, '~confidence_threshold',0.8)

    def _sub_cb(self, msg):
        if len(msg.confidence) > 0:
            speech_conf = msg.confidence[0]
            if speech_conf < self.conf_thresh:
                return

        speech_text = msg.transcript[0]
        self.sound_client.say(
            speech_text,
        )


if __name__ == '__main__':
    node = ros_compat.init_node('parrot_node')
    ParrotNode(node)
    ros_compat.spin(node)
