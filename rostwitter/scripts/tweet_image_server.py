#!/usr/bin/env python
#! -*- coding: utf-8 -*-

import os
import sys
import time

import cv2
import cv_bridge
import rospkg
import rospy
from sound_play.libsoundplay import SoundClient

from rostwitter.twitter import Twitter
from rostwitter.util import load_oauth_settings

from rostwitter.srv import Tweet
from rostwitter.srv import TweetResponse
from sensor_msgs.msg import Image


class TweetImageServer(object):
    def __init__(self):
        self.pack = rospkg.RosPack()
        self.bridge = cv_bridge.CvBridge()
        self.image_path = rospy.get_param(
            '~image_path', '/tmp/tweet_image_server.png')
        self.image_timeout = rospy.get_param('~image_timeout', 5)
        account_info = rospy.get_param(
            '~account_info', '/var/lib/robot/account.yaml')
        ckey, csecret, akey, asecret = load_oauth_settings(account_info)
        if not ckey or not csecret or not akey or not asecret:
            sys.exit(1)

        self.api = Twitter(
            consumer_key=ckey,
            consumer_secret=csecret,
            access_token_key=akey,
            access_token_secret=asecret)
        self.client = SoundClient(
            blocking=True, sound_action='robotsound_jp')
        self.server = rospy.Service(
            '~tweet', Tweet, self._service_cb)

    def _service_cb(self, req):
        ret = None
        success = True
        if req.image:
            os.remove(self.image_path)
            self.sub = rospy.Subscriber(
                req.image_topic_name, Image, self._image_cb)

        if req.warning and req.speak:
            if req.warning_time <= 0:
                warning_text = 'ぜろ'
                req.warning_time = 0
            elif req.warning_time == 1:
                warning_text = 'いち'
            elif req.warning_time == 2:
                warning_text = 'に'
            elif req.warning_time == 3:
                warning_text = 'さん'
            elif req.warning_time == 4:
                warning_text = 'よん'
            elif req.warning_time == 5:
                warning_text = 'ご'
            elif req.warning_time == 6:
                warning_text = 'ろく'
            elif req.warning_time == 7:
                warning_text = 'なな'
            elif req.warning_time == 8:
                warning_text = 'はち'
            elif req.warning_time == 9:
                warning_text = 'きゅう'
            elif req.warning_time >= 10:
                warning_text = 'じゅう'
                req.warning_time = 10
            else:
                warning_text = 'さん'
                req.warning_time = 3
            warning_text = warning_text + 'びょうまえ'
            if req.warning_time > 0:
                time.sleep(req.warning_time)
            wave_path = os.path.join(
                self.pack.get_path('jsk_pr2_startup'),
                'jsk_pr2_lifelog/camera.wav')
            self.client.playWave(wave_path)

        if req.image:
            now = rospy.Time.now()
            while ((rospy.Time.now() - now).to_sec() < self.image_timeout
                    and not os.path.exists(self.image_path)):
                time.sleep(0.1)
            if os.path.exists(self.image_path):
                ret = self.api.post_media(req.text, self.image_path)
            else:
                rospy.logerr('cannot find image: {}'.format(self.image_path))
                ret = self.api.post_update(req.text)
            self.sub.unregister()
            del self.sub
        else:
            ret = self.api.post_update(req.text)

        if not ret or 'errors' in ret:
            success = False
            rospy.logerr('Failed to post: {}'.format(ret))
        if success and req.speak:
            self.client.say('ついーとしました')

        res = TweetResponse(success=success)
        return res

    def _image_cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg)
        cv2.imwrite(self.image_path, img)


if __name__ == '__main__':
    rospy.init_node('tweet_image_server')
    server = TweetImageServer()
    rospy.spin()
