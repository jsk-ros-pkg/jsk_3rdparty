#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import os
import rospy
import tempfile
import unittest

from voice_text.srv import TextToSpeech


class TestVoiceText(unittest.TestCase):
    def test_wave(self):
        text_to_speech = rospy.ServiceProxy(
            "voice_text/text_to_speech", TextToSpeech)
        try:
            text_to_speech.wait_for_service(10)
        except rospy.ROSException:
            self.fail("Service 'voice_text/text_to_speech' not advertised")

        with tempfile.NamedTemporaryFile() as tf:
            tf.write("test speech")
            tf.flush()
            text_path = tf.name
            self.assertTrue(
                os.path.exists(text_path), "text file not generated")

            with tempfile.NamedTemporaryFile() as wf:
                wave_path = wf.name
            self.assertFalse(
                os.path.exists(wave_path), "wave file already exists")

            res = text_to_speech(text_path=text_path, wave_path=wave_path)
            self.assertTrue(res.ok, "Response not OK")
            self.assertTrue(
                os.path.exists(wave_path), "wave file not generated")


if __name__ == '__main__':
    import rostest
    rostest.rosrun("voice_text", "test_voice_text", TestVoiceText)
