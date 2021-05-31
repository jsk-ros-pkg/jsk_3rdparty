#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import pya3rt
import re
from std_msgs.msg import String
from speech_recognition_msgs.msg import SpeechRecognitionCandidates

class TalkA3rt(object):
    def __init__(self):
        a3rtAPIKEY = rospy.get_param('~a3rtAPIKEY', None)
        if a3rtAPIKEY is not None:
            apikey = a3rtAPIKEY
        else:
            rospy.logerr('param: chaplus APIKEY is not correctly set.')
            sys.exit(1)

        self.client = pya3rt.TalkClient(apikey)

        self.talka3rt_pub = rospy.Publisher('chatAPI/a3rt', String, queue_size=1)
        rospy.Subscriber("/Tablet/voice", SpeechRecognitionCandidates, self.sr_cb)

    def sr_cb(self, msg):
        result = msg.transcript[0]
        #Pick out the necessary parts of msg and make a sentence.
        word = re.findall('\s(.*?)\|', result)
        words = "".join(word)
        #In the case of a single word, exclude [num(1, 2, etc.)], and apply as is.
        if words == "":
            words=result[4:]
        print(words)

        speech = self.client.talk(words)
        try:
            answer = speech["results"][0]["reply"].encode("utf-8")
        except:
            #Add the patterns you want answered as needed.
            if speech == "おやすみ":
                answer = "おやすみ"
            elif speech == "おやすみなさい":
                answer = "おやすみなさい"
            else:
                answer = "ごめんなさい、よくわからないです"
        print("a3rt: "+answer)
        self.talka3rt_pub.publish(answer)

if __name__ == '__main__':
    rospy.init_node('a3rt_publisher')
    TalkA3rt()
    rospy.spin()
