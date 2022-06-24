#!/usr/bin/env python

import rospy
import roslib
import unittest

from spatial_audio import SpatialAudioClient
from geometry_msgs.msg import Pose

PKG = 'spatial_audio_demo'
roslib.load_manifest(PKG)


class TestSpatialAudio(unittest.TestCase):

    def test_spatial_audio_client(self):

        source_frame_id = rospy.get_param('~source_frame_id')
        stream_topic_audio = rospy.get_param('~stream_topic_audio')
        stream_topic_info = rospy.get_param('~stream_topic_info')
        auto_play = rospy.get_param('~auto_play')

        client = SpatialAudioClient()
        client.wait_for_server()

        if auto_play:
            success_add, message, audio_source_id = client.add_audio_source(
                source_frame_id,
                stream_topic_audio=stream_topic_audio,
                stream_topic_info=stream_topic_info,
                auto_play=True)
            success_update, message = client.update_audio_source(
                audio_source_id,
                source_frame_id,
                Pose(),
                stream_topic_audio=stream_topic_audio,
                stream_topic_info=stream_topic_info,
            )
            success_stop, message = client.stop_audio_source(audio_source_id)
            success_remove, message = client.remove_audio_source(
                audio_source_id)
            self.assertTrue(success_add, msg='Failed at add_audio_source')
            self.assertTrue(success_update, msg='Failed at update_audio_source')
            self.assertTrue(success_stop, msg='Failed at stop_audio_source')
            self.assertTrue(
                success_remove, msg='Failed at remove_audio_source')
        else:
            success_add, message, audio_source_id = client.add_audio_source(
                source_frame_id,
                stream_topic_audio=stream_topic_audio,
                stream_topic_info=stream_topic_info,
                auto_play=False)
            success_play, message = client.play_audio_source(audio_source_id)
            success_update, message = client.update_audio_source(
                audio_source_id,
                source_frame_id,
                Pose(),
                stream_topic_audio=stream_topic_audio,
                stream_topic_info=stream_topic_info,
            )
            success_stop, message = client.stop_audio_source(audio_source_id)
            success_remove, message = client.remove_audio_source(
                audio_source_id)
            self.assertTrue(success_add, msg='Failed at add_audio_source')
            self.assertTrue(success_play, msg='Failed at play_audio_source')
            self.assertTrue(success_update, msg='Failed at update_audio_source')
            self.assertTrue(success_stop, msg='Failed at stop_audio_source')
            self.assertTrue(
                success_remove, msg='Failed at remove_audio_source')


if __name__ == '__main__':
    import rostest
    rospy.init_node('test_spatial_audio')
    rostest.rosrun(PKG, 'test_spatial_audio', TestSpatialAudio)
