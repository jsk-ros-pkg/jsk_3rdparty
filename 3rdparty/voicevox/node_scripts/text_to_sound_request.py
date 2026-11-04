#!/usr/bin/env python
# -*- coding:utf-8 -*-

"""Relay the text (std_msgs/String) it subscribes on text to sound_play
as a SoundRequest to say on robotsound, in the voice ~speaker
(empty: VOICEVOX_DEFAULT_SPEAKER_ID of sound_play)"""

import os

from std_msgs.msg import String


def sound_request(SoundRequest, text, speaker):
    req = SoundRequest()
    req.sound = SoundRequest.SAY
    req.command = SoundRequest.PLAY_ONCE
    req.volume = 1.0
    req.arg = text
    req.arg2 = str(speaker)
    return req


if __name__ == '__main__':
    if os.environ.get('ROS_VERSION', '1') == '1':
        import rospy
        from sound_play.msg import SoundRequest
        rospy.init_node('text_to_sound_request')
        pub = rospy.Publisher('robotsound', SoundRequest, queue_size=10)
        # ~speaker is read for each message, so that it can be changed at runtime
        rospy.Subscriber('text', String, lambda msg: pub.publish(
            sound_request(SoundRequest, msg.data, rospy.get_param('~speaker', ''))))
        rospy.spin()
    else:
        import rclpy
        from rcl_interfaces.msg import ParameterDescriptor
        from sound_play_msgs.msg import SoundRequest
        rclpy.init()
        node = rclpy.create_node('text_to_sound_request')
        # speaker is an id (int) or a name (str)
        node.declare_parameter('speaker', '', ParameterDescriptor(dynamic_typing=True))
        pub = node.create_publisher(SoundRequest, 'robotsound', 10)
        node.create_subscription(String, 'text', lambda msg: pub.publish(
            sound_request(SoundRequest, msg.data, node.get_parameter('speaker').value)), 10)
        rclpy.spin(node)
