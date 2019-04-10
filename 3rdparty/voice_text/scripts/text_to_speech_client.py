#!/usr/bin/env python

import sys
import os
import rospy
from voice_text.srv import *

def text_to_speech_client(text_path, wave_path):
    rospy.wait_for_service('/voice_text/text_to_speech')
    try:
        text_to_speech = rospy.ServiceProxy('/voice_text/text_to_speech', TextToSpeech)
        # print "Requesting %s -> %s"%(text_path, wave_path)
        res = text_to_speech(text_path, wave_path)
        return res.ok
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [text_path wave_path]"%sys.argv[0]

if __name__ == "__main__":
    argv = rospy.myargv()
    try:
        text_path = str(argv[1])
        wave_path = str(argv[2])
    except:
        raise TypeError("incorrect usage\n%s"%usage())
    print "text_path: %s  wave_path: %s  ok: %s"%(text_path, wave_path, text_to_speech_client(text_path, wave_path))
