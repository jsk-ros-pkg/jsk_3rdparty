# -*- encoding: utf-8 -*-

import rospy
import tf2_ros
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TransformStamped

import threading

from spatial_audio import SpatialAudioClient
from sound_play.libsoundplay import SoundClient


def is_button_pressed(pre_msg, msg, index):

    if len(pre_msg.buttons) <= index or len(msg.buttons) <= index:
        return False

    if pre_msg.buttons[index] == 0 and msg.buttons[index] == 1:
        return True
    else:
        return False


class SoundNavigatorNode(object):

    def __init__(self):

        self._spatial_audio_client = SpatialAudioClient()
        self._sound_client = SoundClient()

        self._spatial_audio_client.wait_for_server()

        # tf
        self._tf_broadcaster = tf2_ros.TransformBroadcaster()

        # parameters
        self._head_frame_id = rospy.get_param(
            '~head_frame_id')
        self._frame_id_target_sound = rospy.get_param(
            '~frame_id_target_sound')
        self._target_sound_stream_topic_audio = rospy.get_param(
            '~target_sound_stream_topic_audio')
        self._target_sound_stream_topic_info = rospy.get_param(
            '~target_sound_stream_topic_info')
        self._axis_x_target_sound = rospy.get_param(
            '~axis_x_target_sound')
        self._axis_x_inverted_target_sound = rospy.get_param(
            '~axis_x_inverted_target_sound')
        self._axis_y_target_sound = rospy.get_param(
            '~axis_y_target_sound')
        self._axis_y_inverted_target_sound = rospy.get_param(
            '~axis_y_inverted_target_sound')
        self._button_watch_out = rospy.get_param('~button_watch_out')
        self._button_stop = rospy.get_param('~button_stop')

        # state of target audio source
        self._target_sound_playing = False
        # position of target audio source
        self._target_sound_lock = threading.Lock()
        self._target_sound_position_x = 0.0
        self._target_sound_position_y = 0.0
        self._target_sound_position_z = 0.0

        # add audio source
        rospy.loginfo('target_sound_stream_topic_audio: {}'.format(
            self._target_sound_stream_topic_audio))
        rospy.loginfo('target_sound_stream_topic_info: {}'.format(
            self._target_sound_stream_topic_info))
        succes, message, self._audio_source_id_target_sound = self._spatial_audio_client.add_audio_source(
            source_frame_id=self._frame_id_target_sound,
            stream_topic_audio=self._target_sound_stream_topic_audio,
            stream_topic_info=self._target_sound_stream_topic_info,
            auto_play=False
        )

        # Start broadcaster timer
        self._timer_broadcast = rospy.Timer(
            rospy.Duration(0.1),
            self.broadcast_transform_target_sound
        )

        # Start subscriber
        self._pre_msg_joy = rospy.wait_for_message('~input_joy', Joy)
        self._sub_joy = rospy.Subscriber('~input_joy', Joy, self.callback_joy, queue_size=1)

    def broadcast_transform_target_sound(self, event):

        with self._target_sound_lock:
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = self._head_frame_id
            t.child_frame_id = self._frame_id_target_sound
            t.transform.translation.x = self._target_sound_position_x
            t.transform.translation.y = self._target_sound_position_y
            t.transform.translation.z = self._target_sound_position_z
            t.transform.rotation.w = 1.0
            self._tf_broadcaster.sendTransform(t)

    def update_target_sound_position(self, axis_x, axis_y):

        with self._target_sound_lock:
            self._target_sound_position_x = axis_x
            self._target_sound_position_y = axis_y

    def callback_joy(self, msg_joy):

        raw_axis_x = msg_joy.axes[self._axis_x_target_sound]
        raw_axis_y = msg_joy.axes[self._axis_y_target_sound]
        rospy.loginfo('raw_axis_x: {}, raw_axis_y: {}'.format(
            raw_axis_x, raw_axis_y))

        if not self._axis_x_inverted_target_sound \
                and not self._axis_y_inverted_target_sound:
            self.update_target_sound_position(
                raw_axis_x,
                raw_axis_y
            )
        elif not self._axis_x_inverted_target_sound \
                and self._axis_y_inverted_target_sound:
            self.update_target_sound_position(
                raw_axis_x,
                -raw_axis_y
            )
        elif self._axis_x_inverted_target_sound \
                and not self._axis_y_inverted_target_sound:
            self.update_target_sound_position(
                -raw_axis_x,
                raw_axis_y
            )
        elif self._axis_x_inverted_target_sound \
                and self._axis_y_inverted_target_sound:
            self.update_target_sound_position(
                -raw_axis_x,
                -raw_axis_y
            )

        if not self._target_sound_playing and \
                (raw_axis_x != 0 or raw_axis_y != 0):
            self._target_sound_playing = True
            self._spatial_audio_client.play_audio_source(
                self._audio_source_id_target_sound)

        if self._target_sound_playing and \
                (raw_axis_x == 0 and raw_axis_x == 0):
            self._target_sound_playing = False
            self._spatial_audio_client.stop_audio_source(
                self._audio_source_id_target_sound)

        if is_button_pressed(
                self._pre_msg_joy,
                msg_joy,
                self._button_watch_out):
            rospy.loginfo('watch out.')
            self._sound_client.say('Watch out.')

        if is_button_pressed(
                self._pre_msg_joy,
                msg_joy,
                self._button_stop):
            rospy.loginfo('stop.')
            self._sound_client.say('Stop')

        self._pre_msg_joy = msg_joy
