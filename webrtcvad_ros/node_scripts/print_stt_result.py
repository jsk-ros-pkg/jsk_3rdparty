#!/usr/bin/env python

import rospy
from speech_recognition_msgs.msg import SpeechRecognitionCandidates

rospy.init_node('hoge')

def callback(msg):
    rospy.loginfo('result: {}'.format(msg.transcript[0]))

sub = rospy.Subscriber('speech_to_text',SpeechRecognitionCandidates,callback)
rospy.spin()
