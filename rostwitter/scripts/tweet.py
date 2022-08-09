#!/usr/bin/env python

import base64
import os
import re
import sys

import rospy
from std_msgs.msg import String

from rostwitter.twitter import Twitter
from rostwitter.util import load_oauth_settings

# https://stackoverflow.com/questions/12315398/check-if-a-string-is-encoded-in-base64-using-python
def isBase64(s):
    try:
        return base64.b64encode(base64.b64decode(s)) == s
    except Exception as e:
        print(e)
        return False

class Tweet(object):
    def __init__(self):
        account_info = rospy.get_param(
            'account_info', '/var/lib/robot/account.yaml')
        ckey, csecret, akey, asecret = load_oauth_settings(account_info)
        if not ckey or not csecret or not akey or not asecret:
            sys.exit(1)

        self.api = Twitter(
            consumer_key=ckey,
            consumer_secret=csecret,
            access_token_key=akey,
            access_token_secret=asecret)
        self.sub = rospy.Subscriber("tweet", String, self.tweet_cb)

    def tweet_cb(self, msg):
        message = msg.data
        rospy.loginfo(rospy.get_name() + " sending %s", message[0:256])

        # search word start from / and end with {.jpeg,.jpg,.png,.gif}
        m = re.search('/\S+\.(jpeg|jpg|png|gif)', message)
        ret = None
        if m:
            filename = m.group(0)
            message = re.sub(filename, "", message)
            if os.path.exists(filename):
                rospy.loginfo(
                    rospy.get_name() + " tweet %s with file %s",
                    message, filename)
                ret = self.api.post_media(message[0:116], filename)
            else:
                rospy.logerr(rospy.get_name() + " %s could not find", filename)

        # search base64 encoding string
        m = re.search('/9j.*$', message)  # jpeg image starts from /9j ????
        if m:
            image = m.group(0)
            message = re.sub("/9j.*$", "", message)
            if isBase64(image):
                rospy.loginfo(
                    rospy.get_name() + " tweet %s with base64 image %s",
                    message, image[0:128])
                ret = self.api.post_media(message[0:116], image)
            else:
                rospy.logerr(rospy.get_name() + " %s is not base64 string", image)

        # post message if not media found
        if m == None:
            ret = self.api.post_update(message[0:140])

        # show results
        if ret and 'errors' in ret:
            rospy.logerr('Failed to post: {}'.format(ret))
        # seg faults if message is longer than 140 byte ???
        rospy.loginfo(rospy.get_name() + " receiving %s", ret)


if __name__ == '__main__':
    rospy.init_node('rostwitter', anonymous=True)
    app = Tweet()
    rospy.spin()
