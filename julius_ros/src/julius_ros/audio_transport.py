#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import rospy
import struct
from julius_ros.transport import SocketTransport
from audio_common_msgs.msg import AudioData


class AudioTransport(SocketTransport):
    def __init__(self, host, port, max_retry, topic_name):
        super(AudioTransport, self).__init__(host, port, max_retry)
        self.topic_name = topic_name
        self.sub_audio = None

    def start(self):
        self.sub_audio = rospy.Subscriber(self.topic_name, AudioData, self.audio_cb)
        super(AudioTransport, self).start()

    def join(self):
        super(AudioTransport, self).join()
        if self.sub_audio:
            self.sub_audio.unregister()

    def audio_cb(self, msg):
        header = struct.pack('i', len(msg.data))
        self.send(header + msg.data)
