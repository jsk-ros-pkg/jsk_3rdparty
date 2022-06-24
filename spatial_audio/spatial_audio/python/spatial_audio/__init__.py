# -*- encoding: utf-8 -*-

import rospy
from spatial_audio_msgs.srv import AddSpatialAudio
from spatial_audio_msgs.srv import AddSpatialAudioRequest
from spatial_audio_msgs.srv import TriggerSpatialAudio
from spatial_audio_msgs.srv import TriggerSpatialAudioRequest
from spatial_audio_msgs.srv import UpdateSpatialAudio
from spatial_audio_msgs.srv import UpdateSpatialAudioRequest
from std_srvs.srv import Trigger
from std_srvs.srv import TriggerRequest
from spatial_audio_msgs.msg import AudioSourceArray
from geometry_msgs.msg import Pose


class SpatialAudioClient(object):

    def __init__(self):

        self._client_add_audio_source = rospy.ServiceProxy(
            '/spatial_audio_server_node/add_audio_source', AddSpatialAudio)
        self._client_update_audio_source = rospy.ServiceProxy(
            '/spatial_audio_server_node/update_audio_source', UpdateSpatialAudio)
        self._client_remove_audio_source = rospy.ServiceProxy(
            '/spatial_audio_server_node/remove_audio_source', TriggerSpatialAudio)
        self._client_remove_all_audio_source = rospy.ServiceProxy(
            '/spatial_audio_server_node/remove_all_audio_source', Trigger)
        self._client_play_audio_source = rospy.ServiceProxy(
            '/spatial_audio_server_node/play_audio_source', TriggerSpatialAudio)
        self._client_stop_audio_source = rospy.ServiceProxy(
            '/spatial_audio_server_node/stop_audio_source', TriggerSpatialAudio)

    def wait_for_server(self, timeout=rospy.Duration(10)):

        rospy.wait_for_service(
            '/spatial_audio_server_node/add_audio_source', timeout)
        rospy.wait_for_service(
            '/spatial_audio_server_node/update_audio_source', timeout)
        rospy.wait_for_service(
            '/spatial_audio_server_node/remove_audio_source', timeout)
        rospy.wait_for_service(
            '/spatial_audio_server_node/remove_all_audio_source', timeout)
        rospy.wait_for_service(
            '/spatial_audio_server_node/play_audio_source', timeout)
        rospy.wait_for_service(
            '/spatial_audio_server_node/stop_audio_source', timeout)

    def get_audio_source_array(self, timeout=rospy.Duration(1.0)):

        try:
            msg = rospy.wait_for_message(
                '/spatial_audio_server_node/audio_source_array', AudioSourceArray, timeout=timeout)
            return msg
        except:
            return None

    def add_audio_source(self,
                         source_frame_id,
                         source_pose=Pose(),
                         stream_topic_audio="",
                         stream_topic_info="",
                         auto_play=False):

        req = AddSpatialAudioRequest()
        req.audio_source.source_frame_id = source_frame_id
        req.audio_source.source_pose = source_pose
        req.audio_source.stream_topic_audio = stream_topic_audio
        req.audio_source.stream_topic_info = stream_topic_info
        req.auto_play = auto_play
        res = self._client_add_audio_source(req)
        return res.success, res.message, res.audio_source_id

    def update_audio_source(self,
                            audio_source_id,
                            source_frame_id,
                            source_pose=Pose(),
                            stream_topic_audio="",
                            stream_topic_info=""):

        req = UpdateSpatialAudioRequest()
        req.audio_source_id = audio_source_id
        req.audio_source.source_frame_id = source_frame_id
        req.audio_source.source_pose = source_pose
        req.audio_source.stream_topic_audio = stream_topic_audio
        req.audio_source.stream_topic_info = stream_topic_info
        res = self._client_update_audio_source(req)
        return res.success, res.message

    def remove_audio_source(self, audio_source_id):

        req = TriggerSpatialAudioRequest()
        req.audio_source_id = audio_source_id
        res = self._client_remove_audio_source(req)
        return res.success, res.message

    def remove_all_audio_source(self):

        req = TriggerRequest()
        res = self._client_remove_all_audio_source(req)
        return res.success, res.message

    def play_audio_source(self, audio_source_id):

        req = TriggerSpatialAudioRequest()
        req.audio_source_id = audio_source_id
        res = self._client_play_audio_source(req)
        return res.success, res.message

    def stop_audio_source(self, audio_source_id):

        req = TriggerSpatialAudioRequest()
        req.audio_source_id = audio_source_id
        res = self._client_stop_audio_source(req)
        return res.success, res.message
