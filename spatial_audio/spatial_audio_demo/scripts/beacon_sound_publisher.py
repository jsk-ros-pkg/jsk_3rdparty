#!/usr/bin/env python
# -*- coding:utf-8 -*-

import wave
import rospy
import sys
import os
import numpy as np
from dynamic_reconfigure.server import Server
from sound_navigator.cfg import BeaconSoundPublisherConfig
from audio_common_msgs.msg import AudioData, AudioInfo


def update_wave(base_wave, rhythm_ratio=1.0, volume=1.0):

    # 多分遅いので直したい。
    base_wave_vol = volume * np.frombuffer(base_wave, dtype='int16')
    resampled_length = int(base_wave_vol.shape[0] / rhythm_ratio)
    resampled_wave = np.zeros((resampled_length,))
    for index in range(resampled_length):
        resampled_wave[index] = base_wave_vol[int(index*1.0*rhythm_ratio)]
    return resampled_wave.astype('int16').tobytes()


class Node(object):

    def callback(self, config, level):

        self.rhythm_ratio = config.rhythm_ratio
        self.volume = config.volume
        return config

    def __init__(self):

        # rosparam
        base_sound_file = rospy.get_param('~base_sound_file')
        self.buffer_size = rospy.get_param('~buffer_size', 4096)

        # dynamic param
        self.rhythm_ratio = 1
        self.volume = 1

        self.srv_dp = Server(BeaconSoundPublisherConfig, self.callback)

        # message
        self.msg_data = AudioData()
        self.msg_info = AudioInfo()

        # load sound file from
        basename, ext = os.path.splitext(base_sound_file)
        if ext == '.wav' or ext == '.wave':
            wf = wave.open(base_sound_file, 'rb')
            channels = wf.getnchannels()
            width = wf.getsampwidth()
            framerate = wf.getframerate()
            framenum = wf.getnframes()
            data = wf.readframes(framenum)
            wf.close()

            if not (channels == 1 and width == 2 and framerate == 16000):
                rospy.logerr('Currently only s16le 16000hz is supported.')
                sys.exit(1)
            if self.buffer_size % width > 0:
                rospy.logerr('buffer_size must be multiple of wave_width')
                sys.exit(1)

            self.msg_info.channels = channels
            self.msg_info.sample_rate = framerate
            if width == 1:
                self.msg_info.sample_format = 'S8'
            elif width == 2:
                self.msg_info.sample_format = 'S16LE'

            self.framerate = framerate

            self.base_wave_data = data

        else:
            rospy.logerr('Unsupported file type: {}'.format(ext))
            return

    def spin(self):

        buffer_array = b''
        # publishers
        publisher_data = rospy.Publisher('~audio', AudioData, queue_size=1)
        publisher_info = rospy.Publisher('~info', AudioInfo, queue_size=1)

        rate = rospy.Rate(10)
        duration_published = rospy.Duration(0)
        time_start = rospy.Time.now()
        while not rospy.is_shutdown():

            if duration_published - (rospy.Time.now() - time_start) > rospy.Duration(0.1):
                rate.sleep()
                continue

            if len(buffer_array) < self.buffer_size:
                wave_single = update_wave(
                    self.base_wave_data, self.rhythm_ratio, self.volume)
                while len(buffer_array) < self.buffer_size:
                    buffer_array += wave_single

            self.msg_data.data = buffer_array[:self.buffer_size]
            buffer_array = buffer_array[self.buffer_size:]
            duration_published += rospy.Duration(
                (self.buffer_size/2)*1.0/self.framerate)
            publisher_data.publish(self.msg_data)
            publisher_info.publish(self.msg_info)


if __name__ == '__main__':

    rospy.init_node('beacon_sound_publisher')
    node = Node()
    node.spin()
