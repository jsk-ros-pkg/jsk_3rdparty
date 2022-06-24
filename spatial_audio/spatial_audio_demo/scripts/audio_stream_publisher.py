#!/usr/bin/env python
# -*- coding:utf-8 -*-

import sys
import time
import wave

import rospy
from audio_common_msgs.msg import AudioData
from audio_common_msgs.msg import AudioInfo


def main():

    rospy.init_node('audio_stream_publisher')

    # parameters
    loop_rate = rospy.get_param('~loop_rate', 100)
    is_loop = rospy.get_param('~is_loop', False)
    filename = rospy.get_param('~filename')
    buffer_size = rospy.get_param('~buffer_size', 4096)
    max_precede = rospy.get_param('~max_precede', 10.0)

    # publishers
    publisher_data = rospy.Publisher('~audio', AudioData, queue_size=1)
    publisher_info = rospy.Publisher('~info', AudioInfo, queue_size=1)

    # message
    msg_data = AudioData()
    msg_info = AudioInfo()

    # open wavefile
    wavefile = wave.open(filename, 'r')
    wave_channels = wavefile.getnchannels()
    wave_width = wavefile.getsampwidth()
    wave_framerate = wavefile.getframerate()
    wave_framenum = wavefile.getnframes()
    wave_data = wavefile.readframes(wave_framenum)

    infostr = (
        '== Wave file info ==\n' +
        'channels: {}\n' +
        'sample size [byte]: {}\n' +
        'sampling rate [hz]: {}\n' +
        'number of frames: {} ( {} secs )').format(wave_channels, wave_width, wave_framerate, wave_framenum, wave_framenum*1.0/wave_framerate)
    rospy.loginfo(infostr)

    if wave_channels != 1 or not (wave_width == 1 or wave_width == 2):
        rospy.logerr(
            'Currently 8bit or 16bit monaural wave files are supported.')
        sys.exit(1)
    if buffer_size % wave_width > 0:
        rospy.logerr('buffer_size must be multiple of wave_width')
        sys.exit(1)

    msg_info.channels = wave_channels
    msg_info.sample_rate = wave_framerate
    if wave_width == 1:
        msg_info.sample_format = 'S8'
    else:
        msg_info.sample_format = 'S16LE'
    publisher_info.publish(msg_info)

    r = rospy.Rate(loop_rate)
    if is_loop:
        time_played = 0.0  # sec
        time_start = time.time()
        index = 0
        while not rospy.is_shutdown():
            # return to first if index is out of wave_data
            if index > wave_framenum:
                index = 0
            # preparing msg data
            if (wave_framenum - index) < buffer_size:
                msg_data.data = wave_data[index:]
                time_played += 1.0 * (wave_framenum - index) / \
                    (wave_width * wave_framerate)
            else:
                msg_data.data = wave_data[index:index+buffer_size]
                time_played += 1.0 * buffer_size / \
                    (wave_width * wave_framerate)
            # sleep if played time is ahead of passed time
            time_passed = time.time() - time_start
            while time_played - time_passed > max_precede:
                time_passed = time.time() - time_start
                if rospy.is_shutdown():
                    return
                r.sleep()
            # publishing data
            publisher_data.publish(msg_data)
            publisher_info.publish(msg_info)
            # debug print
            rospy.logdebug('time played: {}, time passed {}'.format(
                time_played, time_passed))
            # increase index
            index += buffer_size
    else:
        time_played = 0.0  # sec
        time_start = time.time()
        index = 0
        while not rospy.is_shutdown():
            # return to first if index is out of wave_data
            if index > wave_framenum:
                break
            # preparing msg data
            if (wave_framenum - index) < buffer_size:
                msg_data.data = wave_data[index:]
                time_played += 1.0 * (wave_framenum - index) / \
                    (wave_width * wave_framerate)
            else:
                msg_data.data = wave_data[index:index+buffer_size]
                time_played += 1.0 * buffer_size / \
                    (wave_width * wave_framerate)
            # sleep if played time is ahead of passed time
            time_passed = time.time() - time_start
            while time_played - time_passed > max_precede:
                time_passed = time.time() - time_start
                if rospy.is_shutdown():
                    return
                r.sleep()
            # publishing data
            publisher_data.publish(msg_data)
            publisher_info.publish(msg_info)
            # debug print
            rospy.logdebug('time played: {}, time passed {}'.format(
                time_played, time_passed))
            # increase index
            index += buffer_size


if __name__ == '__main__':
    main()
