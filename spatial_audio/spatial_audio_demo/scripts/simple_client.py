#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from spatial_audio import SpatialAudioClient


def main():

    rospy.init_node('simple_client')

    source_frame_id = rospy.get_param('~source_frame_id', 'source_sinwave')
    stream_topic_audio = rospy.get_param('~stream_topic_audio')
    stream_topic_info = rospy.get_param('~stream_topic_info')

    client = SpatialAudioClient()
    client.wait_for_server()

    success, message, audio_source_id = client.add_audio_source(
        source_frame_id, stream_topic_audio=stream_topic_audio, stream_topic_info=stream_topic_info, auto_play=True)

    rospy.loginfo('success: {}, message: {}, audio_source_id: {}'.format(
        success, message, audio_source_id))


if __name__ == '__main__':
    ret = main()
