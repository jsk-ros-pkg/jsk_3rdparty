#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import time
import numpy as np
import math
import tf2_ros
from audio_common_msgs.msg import AudioData, AudioInfo


def generateSin(A, f0, fs, duration):
    return A * np.sin(2 * np.pi * f0 * np.arange(int(duration*fs)) / fs)


def generateBeepSingle(A, f0, fs, duration, ratio_sleep):
    duration_sound = duration * (1 - ratio_sleep)
    duration_sleep = duration * ratio_sleep
    if type(f0) is not list:
        return np.concatenate([generateSin(A, f0, fs, duration_sound), np.zeros(int(duration_sleep*fs))])
    else:
        return np.concatenate([sum([generateSin(A, freq, fs, duration_sound) for freq in f0]) / len(f0),
                               np.zeros(int(duration_sleep*fs))])


def generateBeep(A, f0, f1, fs, duration, ratio_sleep):
    duration_single = 1.0 / f1
    beep_times = int(duration / duration_single)
    return np.concatenate([generateBeepSingle(A, f0, fs, duration_single, ratio_sleep) for x in range(beep_times)])


def main():

    rospy.init_node('beep_publisher')

    #
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # parameters
    loop_rate = rospy.get_param('~loop_rate', 100)
    buffer_size = rospy.get_param('~buffer_size', 4096)
    frame_id_target = rospy.get_param('~frame_id_target', 'target')
    frame_id_source = rospy.get_param('~frame_id_source', 'source')
    range_distance_min = rospy.get_param('~range_distance_min', 0.01)  # [m]
    range_distance_max = rospy.get_param('~range_distance_max', 1.0)  # [m]
    range_frequency_min = rospy.get_param('~range_frequency_min', 1.0)  # [Hz]
    range_frequency_max = rospy.get_param('~range_frequency_max', 5.0)  # [Hz]
    range_steps = rospy.get_param('~steps', 20)

    # publishers
    publisher_data = rospy.Publisher('~audio', AudioData, queue_size=1)
    publisher_info = rospy.Publisher('~info', AudioInfo, queue_size=1)

    # message
    msg_data = AudioData()
    msg_info = AudioInfo()

    # gen a default buffer
    msg_info.channels = 1
    msg_info.sample_rate = 16000
    msg_info.sample_format = 'S16LE'

    #
    distances = [range_distance_min + (range_distance_max - range_distance_min)
                 * index / range_steps for index in range(range_steps+1)]
    frequencies = [range_frequency_max - (range_frequency_max - range_frequency_min)
                   * index / range_steps for index in range(range_steps+1)]
    buffers = [generateBeep(0.1*((2**8) ** 2), [440, 480, 500, 520, 400, 300, 600, 800], freq, msg_info.sampling_rate, 10.0, 0.5).astype(np.uint16).tobytes()
               for freq in frequencies]

    r = rospy.Rate(loop_rate)
    preindex_distance = 0
    index_buf = 0
    buf = np.zeros(buffer_size).astype(np.uint16).tobytes()
    time_played = 0.0
    time_start = time.time()
    while not rospy.is_shutdown():
        try:
            t = tfBuffer.lookup_transform(
                frame_id_target,
                frame_id_source,
                rospy.Time()
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.loginfo(e)
            continue

        # distance を計算
        distance = math.sqrt(t.transform.translation.x ** 2 +
                             t.transform.translation.y ** 2 + t.transform.translation.z ** 2)

        index_distance = 0
        for i, d in enumerate(distances):
            if d > distance:
                break
            else:
                index_distance = i

        if index_distance != preindex_distance:
            rospy.loginfo('distance changed')
            buf = buffers[index_distance]
            index_buf = 0
        preindex_distance = index_distance

        if index_buf > len(buf):
            index_buf = 0

        if (len(buf) - index_buf) < buffer_size:
            msg_data.data = buf[index:]
            time_played += 1.0 * (len(buf) - index_buf) / \
                (msg_info.width * msg_info.sampling_rate)
        else:
            msg_data.data = buf[index_buf:index_buf+buffer_size]
            time_played += 1.0 * buffer_size / \
                (msg_info.width * msg_info.sampling_rate)
        time_passed = time.time() - time_start
        while time_played - time_passed > 0:
            time_passed = time.time() - time_start
            if rospy.is_shutdown():
                return
            r.sleep()
        publisher_data.publish(msg_data)
        publisher_info.publish(msg_info)
        index_buf += buffer_size

        rospy.loginfo("distance: {}, freq: {}".format(
            distance, frequencies[index_distance]))


if __name__ == '__main__':
    main()
