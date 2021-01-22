#!/usr/bin/env python

import os
import re
import sys
import yaml

import rospy
from std_msgs.msg import String

from rostwitter.twitter import Twitter


class Tweet(object):
    def __init__(self):
        self.load_oauth_settings()
        self.api = Twitter(
            consumer_key=self.CKEY,
            consumer_secret=self.CSECRET,
            access_token_key=self.AKEY,
            access_token_secret=self.ASECRET)
        self.sub = rospy.Subscriber("tweet", String, self.tweet_cb)

    def tweet_cb(self, msg):
        message = msg.data
        rospy.loginfo(rospy.get_name() + " sending %s", message)

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
                # ret = self.api.post_update(message)
            else:
                rospy.logerr(rospy.get_name() + " %s could not find", filename)
        else:
            ret = self.api.post_update(message[0:140])
        # seg faults if message is longer than 140 byte ???
        rospy.loginfo(rospy.get_name() + " receiving %s", ret)

    def load_oauth_settings(self):
        account_info = rospy.get_param(
            'account_info', '/var/lib/robot/account.yaml')

        try:
            key = yaml.load(open(account_info))
            self.CKEY = key['CKEY']
            self.CSECRET = key['CSECRET']
            self.AKEY = key['AKEY']
            self.ASECRET = key['ASECRET']
        except IOError as e:
            rospy.logerr(e)
            rospy.logerr('"%s" not found' % account_info)
            rospy.logerr("$ get access token from https://apps.twitter.com/")
            rospy.logerr("cat /var/lib/robot/%s <<EOF" % account_info)
            rospy.logerr("CKEY: xxx")
            rospy.logerr("CSECRET: xxx")
            rospy.logerr("AKEY: xxx")
            rospy.logerr("ASECRET: xxx")
            rospy.logerr("EOF")
            sys.exit(-1)


if __name__ == '__main__':
    rospy.init_node('rostwitter', anonymous=True)
    app = Tweet()
    rospy.spin()
