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
        # base64 messages are noisy, so tweet log is suppressed
        rospy.loginfo(rospy.get_name() + " sending %s",
                      ''.join([message] if len(message) < 128 else message[0:128]+'......'))

        ret = self.api.post_update(message)
        if 'errors' in ret:
            rospy.logerr('Failed to post: {}'.format(ret))
        rospy.loginfo(rospy.get_name() + " receiving %s", ret)


if __name__ == '__main__':
    rospy.init_node('rostwitter', anonymous=True)
    app = Tweet()
    rospy.spin()
