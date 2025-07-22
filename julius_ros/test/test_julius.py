#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


import rospy
import unittest
from speech_recognition_msgs.msg import SpeechRecognitionCandidates


class TestJulius(unittest.TestCase):

    def on_speech(self, msg):
        self.transcripts.append(msg.transcript[0])

    def test_julius(self):
        if rospy.get_param('~dnn', False):
            expected_sentences = [' わかめ 。']
        else:
            expected_sentences = ['わかめ', 'きつね']
        self.transcripts = []
        sub = rospy.Subscriber("speech_to_text", SpeechRecognitionCandidates,
                               self.on_speech)

        start_time = rospy.Time.now()
        while len(self.transcripts) < len(expected_sentences):
            rospy.sleep(1)
            if (rospy.Time.now() - start_time).to_sec() > 60:
                self.fail("Timeout.")

        for s in expected_sentences:
            self.assertTrue(s in self.transcripts)


if __name__ == '__main__':
    import rostest
    rospy.init_node("test_julius")
    rostest.rosrun("julius_ros", "test_julius", TestJulius)
