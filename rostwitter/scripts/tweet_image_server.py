#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import threading
import time

import actionlib
import cv2
import cv_bridge
from dynamic_reconfigure.server import Server
import rospkg
import rospy
from sound_play.libsoundplay import SoundClient

from rostwitter.twitter import Twitter
from rostwitter.util import load_oauth_settings

from rostwitter.cfg import TweetImageServerConfig
from rostwitter.msg import TweetAction
from rostwitter.msg import TweetFeedback
from rostwitter.msg import TweetResult

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

        try:  # sound_play version < 0.3.7 does not support 'sound_action' argument
            self.client = SoundClient(
                blocking=True, sound_action='robotsound_jp')
        except:
            rospy.logwarn("sound_play version < 0.3.7 does not support 'sound_action' argument, so it uses robot_sound, instead of robot_sound_jp")
            self.client = SoundClient(
                blocking=True)
        self.image_topic_name = None
        self.img = {}
        self.lock = threading.Lock()

        self.config_server = Server(
            TweetImageServerConfig, self._config_cb)
        self.server = actionlib.SimpleActionServer(
            '~tweet', TweetAction, self._execute_cb)


    def _execute_cb(self, goal):
        ret = None
        success = True
        if goal.image:
            if os.path.exists(self.image_path):
                os.remove(self.image_path)
            self.image_topic_name = goal.image_topic_name
            with self.lock:
                self.img[self.image_topic_name] = None
            sub = rospy.Subscriber(
                self.image_topic_name, Image,
                self._image_cb)

        if goal.warning and goal.speak:
            if goal.warning_time <= 0:
                warning_text = 'ぜろ'
                goal.warning_time = 0
            elif goal.warning_time == 1:
                warning_text = 'いち'
            elif goal.warning_time == 2:
                warning_text = 'に'
            elif goal.warning_time == 3:
                warning_text = 'さん'
            elif goal.warning_time == 4:
                warning_text = 'よん'
            elif goal.warning_time == 5:
                warning_text = 'ご'
            elif goal.warning_time == 6:
                warning_text = 'ろく'
            elif goal.warning_time == 7:
                warning_text = 'なな'
            elif goal.warning_time == 8:
                warning_text = 'はち'
            elif goal.warning_time == 9:
                warning_text = 'きゅう'
            elif goal.warning_time >= 10:
                warning_text = 'じゅう'
                goal.warning_time = 10
            else:
                warning_text = 'さん'
                goal.warning_time = 3
            warning_text = warning_text + 'びょうまえ'
            self.client.say(warning_text, volume=self.volume)
            if goal.warning_time > 0:
                time.sleep(goal.warning_time)
            wave_path = os.path.join(
                self.pack.get_path('rostwitter'),
                'resource/camera.wav')
            self.client.playWave(wave_path, volume=self.volume)

        if goal.image:
            now = rospy.Time.now()
            while ((rospy.Time.now() - now).to_sec() < self.image_timeout
                    and self.img[self.image_topic_name] is None):
                time.sleep(0.1)
                if self.server.is_preempt_requested():
                    rospy.logerr('tweet image server preempted')
                    self.server.set_preempted()
                    success = False
                    break
                feedback = TweetFeedback(stamp=rospy.Time.now())
                self.server.publish_feedback(feedback)
            if (success and self.img[self.image_topic_name] is not None):
                with self.lock:
                    cv2.imwrite(self.image_path, self.img[self.image_topic_name])
                ret = self.api.post_media(goal.text, self.image_path)
            else:
                rospy.logerr('cannot subscribe image: {}'.format(self.image_topic_name))
                ret = self.api.post_update(goal.text)
            sub.unregister()
        else:
            ret = self.api.post_update(goal.text)

        if not ret or 'errors' in ret:
            success = False
            rospy.logerr('Failed to post: {}'.format(ret))

        res = TweetResult(success=success)
        if success:
            if goal.speak:
                self.client.say('ついーとしました', volume=self.volume)
            self.server.set_succeeded(res)
        else:
            self.server.set_aborted(res)

    def _image_cb(self, msg):
        if self.image_topic_name:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.lock:
                self.img[self.image_topic_name] = img

    def _config_cb(self, config, level):
        self.volume = config.volume
        return config


if __name__ == '__main__':
    rospy.init_node('tweet_image_server')
    server = TweetImageServer()
    rospy.spin()
