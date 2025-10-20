#!/usr/bin/env python

import argparse
import sys
import os
import rospy
from voice_text.srv import *


def text_to_speech_client(text_path, wave_path):
    rospy.wait_for_service('/voice_text/text_to_speech')
    try:
        text_to_speech = rospy.ServiceProxy('/voice_text/text_to_speech', TextToSpeech)
        # print("Requesting %s -> %s"%(text_path, wave_path))
        res = text_to_speech(text_path, wave_path)
        return res.ok
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='')
    parser.add_argument('-eval', '--evaluate')
    parser.add_argument('-o', '--output')
    parser.add_argument('text')
    args = parser.parse_args()

    input_file = args.text
    jptext_file = "/tmp/_voice_text_%s.txt"%os.getpid()
    output_file = "/tmp/_voice_text_%s.wav"%os.getpid()
    if args.output is not None:
        output_file = args.output

    if not os.path.exists(input_file):
        rospy.logerr("%s not found"%input_file)
        sys.exit(1)

    # convert character code to shift JIS
    os.system("nkf -s %s > %s"%(input_file, jptext_file))

    text_to_speech_client(jptext_file, output_file)

    os.remove(jptext_file)
