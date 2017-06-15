#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import os
import rospy
from speech_recognition_msgs.msg import Vocabulary
from speech_recognition_msgs.msg import Grammar
from julius_ros.utils import load_grammar


def register_isolated(name, words):
    pub = rospy.Publisher("/vocabulary", Vocabulary, queue_size=1)
    for i in range(10):
        if rospy.is_shutdown() or pub.get_num_connections() > 0:
            break
        rospy.sleep(1)
        rospy.loginfo("Waiting /vocabulary is advertised")
    if pub.get_num_connections() == 0:
        rospy.logerr("/vocabulary is not advertised")
        return
    voca = Vocabulary(words=words)
    if name:
        voca.name = name
    pub.publish(voca)


def register_grammar(name, path):
    root_dir, name = os.path.dirname(path), os.path.basename(path)
    pub = rospy.Publisher("/grammar", Grammar, queue_size=1)
    for i in range(10):
        if rospy.is_shutdown() or pub.get_num_connections() > 0:
            break
        rospy.sleep(1)
        rospy.loginfo("Waiting /grammar is advertised")
    if pub.get_num_connections() == 0:
        rospy.logerr("/grammar is not advertised")
        return
    gram = load_grammar(root_dir, name)
    if name:
        gram.name = name
    pub.publish(gram)
