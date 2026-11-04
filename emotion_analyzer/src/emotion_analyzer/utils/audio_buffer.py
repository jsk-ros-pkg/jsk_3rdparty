#!/usr/bin/env python3
from __future__ import division

import array
import asyncio
from base64 import b64encode
from io import BytesIO
from threading import Lock
import traceback
import time

from audio_common_msgs.msg import AudioData
from hume import HumeStreamClient
from hume.models.config import BurstConfig
from hume.models.config import ProsodyConfig
from hume import StreamSocket
import numpy as np
from pydub import AudioSegment
import soundfile as sf

try:
    from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat
    from jsk_ros1_ros2_compat.rospy_rclpy_compat import ROS_VERSION
except ImportError:
    # jsk_common < 3.0 has no jsk_ros1_ros2_compat, so this is ROS1 only:
    # forward ros_compat.xxx(node, ...) to rospy.xxx(...)
    import rospy
    ROS_VERSION = 1
    class ros_compat(object):
        create_subscription = staticmethod(lambda node, *args, **kwargs: rospy.Subscriber(*args, **kwargs))
        get_param = staticmethod(lambda node, *args, **kwargs: rospy.get_param(*args, **kwargs))
        is_shutdown = staticmethod(lambda node, *args, **kwargs: rospy.is_shutdown(*args, **kwargs))
        sleep = staticmethod(lambda node, *args, **kwargs: rospy.sleep(*args, **kwargs))

class AudioBuffer(object):

    def __init__(self, node=None, topic_name='~audio',
                 input_sample_rate=16000,
                 window_size=10.0,
                 bitdepth=16,
                 n_channel=1, target_channel=0,
                 get_latest_data=False,
                 discard_data=False,
                 auto_start=False):
        self.node = node
        self.is_subscribing = True
        self.get_latest_data = get_latest_data
        self.discard_data = discard_data
        self._window_size = window_size
        self.audio_buffer_len = int(self._window_size * input_sample_rate)
        self.lock = Lock()
        self.bitdepth = bitdepth
        self.n_channel = n_channel
        self.target_channel = min(self.n_channel - 1, max(0, target_channel))
        self.input_sample_rate = input_sample_rate
        self.type_code = {}
        for code in ['b', 'h', 'i', 'l']:
            self.type_code[array.array(code).itemsize] = code

        self.dtype = self.type_code[self.bitdepth / 8]
        self.audio_buffer = np.array([], dtype=self.dtype)

        self.max_value = 2 ** (self.bitdepth - 1) - 1

        self.topic_name = topic_name

        if auto_start:
            self.subscribe()

    def __len__(self):
        return len(self.audio_buffer)

    @property
    def window_size(self):
        return self._window_size

    @window_size.setter
    def window_size(self, size):
        with self.lock:
            self._window_size = size
            self.audio_buffer_len = int(self._window_size
                                        * self.input_sample_rate)
            self.audio_buffer = np.array([], dtype=self.dtype)

    @staticmethod
    def from_rosparam(node=None, **kwargs):
        n_channel = ros_compat.get_param(node, '~n_channel', 1)
        target_channel = ros_compat.get_param(node, '~target_channel', 0)
        mic_sampling_rate = ros_compat.get_param(node, '~mic_sampling_rate', 16000)
        bitdepth = ros_compat.get_param(node, '~bitdepth', 16)
        return AudioBuffer(node=node, input_sample_rate=mic_sampling_rate,
                           bitdepth=bitdepth,
                           n_channel=n_channel,
                           target_channel=target_channel,
                           **kwargs)

    def subscribe(self):
        self.audio_buffer = np.array([], dtype=self.dtype)
        self.sub_audio = ros_compat.create_subscription(
            self.node, self.topic_name, AudioData, self.audio_cb)

    def unsubscribe(self):
        if ROS_VERSION == 2:
            self.node.destroy_subscription(self.sub_audio)
        else:
            self.sub_audio.unregister()

    def _read(self, size, normalize=False):
        with self.lock:
            if self.get_latest_data:
                audio_buffer = self.audio_buffer[-size:]
            else:
                audio_buffer = self.audio_buffer[:size]
                if self.discard_data:
                    self.audio_buffer = self.audio_buffer[size:]
        if normalize is True:
            audio_buffer = audio_buffer / self.max_value
        return audio_buffer

    def sufficient_data(self, size):
        return len(self.audio_buffer) < size

    def read(self, size=None, wait=False, normalize=False):
        if size is None:
            size = int(self.audio_buffer_len)
        while wait is True \
                and not ros_compat.is_shutdown(self.node) and len(self.audio_buffer) < size:
            ros_compat.sleep(self.node, 0.001)
        return self._read(size, normalize=normalize)

    def close(self):
        try:
            self.unsubscribe()
        except Exception:
            pass
        self.audio_buffer = np.array([], dtype=self.dtype)

    def audio_cb(self, msg):
        audio_buffer = np.frombuffer(msg.data, dtype=self.dtype)
        audio_buffer = audio_buffer[self.target_channel::self.n_channel]
        with self.lock:
            self.audio_buffer = np.append(
                self.audio_buffer, audio_buffer)
            self.audio_buffer = self.audio_buffer[
                -self.audio_buffer_len:]
