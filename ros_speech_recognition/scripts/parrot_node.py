#!/usr/bin/env python

import rospy
from sound_play.libsoundplay import SoundClient

from speech_recognition_msgs.msg import SpeechRecognitionCandidates


class ParrotNode(object):
    def __init__(self):
        self.sub = rospy.Subscriber(
            '~input',
            SpeechRecognitionCandidates,
            self._sub_cb,
            queue_size=1
        )
        tts_action_name = rospy.get_param(
            '~tts_action_name', 'sound_play')
        self.sound_client = SoundClient(
            blocking=True,
            sound_action=tts_action_name,
        )
        self.conf_thresh = rospy.get_param('~confidence_threshold',0.8)

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
    rospy.init_node('parrot_node')
    node = ParrotNode()
    rospy.spin()
