#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
import requests
import json
import re

class TalkChaplus(object):

    def __init__(self):
        chaplusAPIKEY = rospy.get_param('~chaplusAPIKEY', None)
        if chaplusAPIKEY is not None:
            self.url = "https://www.chaplus.jp/v1/chat?apikey=" + chaplusAPIKEY
        else:
            rospy.logerr('param: chaplus APIKEY is not correctly set.')
            sys.exit(1)

        self.talkchaplus_pub = rospy.Publisher('/chatAPI/chaplus', String, queue_size=1)
        rospy.Subscriber("/Tablet/voice", SpeechRecognitionCandidates, self.topic_cb)

        self.headers = {'content-type':'text/json'}

    def topic_cb(self, msg):
        result = msg.transcript[0]
        #Pick out the necessary parts of msg and make a sentence.
        word = re.findall('\s(.*?)\|', result)
        words = "".join(word)
        #In the case of a single word, exclude [num(1, 2, etc.)], and apply as is.
        if words == "":
            words=result[4:]
        print(words)

        #use chaplus
        payload = {'utterance':words}
        res = requests.post(url=self.url, headers=self.headers, data=json.dumps(payload))
        best_res = res.json()['bestResponse']['utterance']

        #publish as rostopic
        answer = String()
        answer.data = best_res
        print("chaplus: "+ best_res)
        self.talkchaplus_pub.publish(answer)

if __name__ == '__main__':
    rospy.init_node('chaplus_publisher')
    TalkChaplus()
    rospy.spin()
