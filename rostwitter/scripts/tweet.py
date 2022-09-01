#!/usr/bin/env python

import os
import re
import sys

import rospy
from std_msgs.msg import String

from rostwitter.twitter import Twitter
from rostwitter.util import load_oauth_settings


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
        rospy.logdebug(rospy.get_name() + " sending %s", message)

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
                # 140 - len("http://t.co/ssssssssss")
                ret = self.api.post_media(message[0:116], filename)
                if 'errors' in ret:
                    rospy.logerr('Failed to post: {}'.format(ret))
                # ret = self.api.post_update(message)
            else:
                rospy.logerr(rospy.get_name() + " %s could not find", filename)
        else:
            ret = self.api.post_update(message[0:140])
            if 'errors' in ret:
                rospy.logerr('Failed to post: {}'.format(ret))
        # seg faults if message is longer than 140 byte ???
        rospy.loginfo(rospy.get_name() + " receiving %s", ret)


if __name__ == '__main__':
    rospy.init_node('rostwitter', anonymous=True)
    app = Tweet()
    rospy.spin()
