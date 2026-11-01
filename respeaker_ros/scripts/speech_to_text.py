#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

try:
    from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat
except ImportError:
    ros_compat = None  # jsk_common < 3.0 (ROS1 only): use the original rospy code

if ros_compat is None:
    import actionlib
    import rospy
try:
    import speech_recognition as SR
except ImportError as e:
    raise ImportError(str(e) + '\nplease try "pip install speechrecognition"')

try:
    from actionlib_msgs.msg import GoalStatus, GoalStatusArray
    from sound_play.msg import SoundRequest, SoundRequestAction, SoundRequestGoal
    # actionlib's status topic and state of an executing goal
    TTS_STATUS_TOPIC, TTS_STATUS_ACTIVE = '/status', GoalStatus.ACTIVE
except ImportError:  # ROS2: the action interface is in sound_play_msgs
    from action_msgs.msg import GoalStatus, GoalStatusArray
    from sound_play_msgs.action import SoundRequest as SoundRequestAction
    TTS_STATUS_TOPIC, TTS_STATUS_ACTIVE = '/_action/status', GoalStatus.STATUS_EXECUTING
from audio_common_msgs.msg import AudioData
from speech_recognition_msgs.msg import SpeechRecognitionCandidates


class SpeechToText(object):
    def __init__(self):
        # format of input audio data
        self.sample_rate = rospy.get_param("~sample_rate", 16000)
        self.sample_width = rospy.get_param("~sample_width", 2)
        # language of STT service
        self.language = rospy.get_param("~language", "ja-JP")
        # ignore voice input while the robot is speaking
        self.self_cancellation = rospy.get_param("~self_cancellation", True)
        # time to assume as SPEAKING after tts service is finished
        self.tts_tolerance = rospy.Duration.from_sec(
            rospy.get_param("~tts_tolerance", 1.0))
        tts_action_names = rospy.get_param(
            '~tts_action_names', ['sound_play'])

        self.recognizer = SR.Recognizer()

        self.tts_action = None
        self.last_tts = None
        self.is_canceling = False
        self.tts_actions = []
        if self.self_cancellation:
            for tts_action_name in tts_action_names:
                tts_action = actionlib.SimpleActionClient(
                    tts_action_name, SoundRequestAction)
                if tts_action.wait_for_server(rospy.Duration(5.0)):
                    self.tts_actions.append(tts_action)
                else:
                    rospy.logerr(
                        "action '{}' is not initialized."
                        .format(tts_action_name))
                self.tts_timer = rospy.Timer(rospy.Duration(0.1), self.tts_timer_cb)

        self.pub_speech = rospy.Publisher(
            "speech_to_text", SpeechRecognitionCandidates, queue_size=1)
        self.sub_audio = rospy.Subscriber("audio", AudioData, self.audio_cb)

    def tts_timer_cb(self, event):
        stamp = event.current_real
        active = False
        for tts_action in self.tts_actions:
            for st in tts_action.action_client.last_status_msg.status_list:
                if st.status == GoalStatus.ACTIVE:
                    active = True
                    break
            if active:
                break
        if active:
            if not self.is_canceling:
                rospy.logdebug("START CANCELLATION")
                self.is_canceling = True
                self.last_tts = None
        elif self.is_canceling:
            if self.last_tts is None:
                self.last_tts = stamp
            if stamp - self.last_tts > self.tts_tolerance:
                rospy.logdebug("END CANCELLATION")
                self.is_canceling = False

    def audio_cb(self, msg):
        if self.is_canceling:
            rospy.loginfo("Speech is cancelled")
            return
        data = SR.AudioData(msg.data, self.sample_rate, self.sample_width)
        try:
            rospy.loginfo("Waiting for result %d" % len(data.get_raw_data()))
            result = self.recognizer.recognize_google(
                data, language=self.language)
            msg = SpeechRecognitionCandidates(
                transcript=[result],
                confidence=[1.0],
            )
            self.pub_speech.publish(msg)
        except SR.UnknownValueError as e:
            rospy.logerr("Failed to recognize: %s" % str(e))
        except SR.RequestError as e:
            rospy.logerr("Failed to recognize: %s" % str(e))


class SpeechToTextCompat(SpeechToText):
    """SpeechToText written with jsk_ros1_ros2_compat (ROS1/ROS2). The
    state of the tts goals is taken from the action's status topic, since
    ROS2 action clients do not keep the status of other clients' goals."""

    def __init__(self, node=None):
        self.node = node
        # format of input audio data
        self.sample_rate = ros_compat.get_param(node, "~sample_rate", 16000)
        self.sample_width = ros_compat.get_param(node, "~sample_width", 2)
        # language of STT service
        self.language = ros_compat.get_param(node, "~language", "ja-JP")
        # ignore voice input while the robot is speaking
        self.self_cancellation = ros_compat.get_param(node, "~self_cancellation", True)
        # time to assume as SPEAKING after tts service is finished
        self.tts_tolerance = ros_compat.get_param(node, "~tts_tolerance", 1.0)  # sec
        tts_action_names = ros_compat.get_param(
            node, '~tts_action_names', ['sound_play'])

        self.recognizer = SR.Recognizer()

        self.tts_action = None
        self.last_tts = None
        self.is_canceling = False
        self.tts_actions = []
        self.tts_status = {}
        if self.self_cancellation:
            for tts_action_name in tts_action_names:
                tts_action = ros_compat.create_simple_action_client(
                    node, tts_action_name, SoundRequestAction)
                if tts_action.wait_for_server(5.0):
                    self.tts_actions.append(tts_action)
                    ros_compat.create_subscription(
                        node, tts_action_name + TTS_STATUS_TOPIC, GoalStatusArray,
                        lambda msg, a=tts_action: self.tts_status.__setitem__(a, msg),
                        queue_size=1, latch=True)
                else:
                    ros_compat.logerr(node,
                        "action '{}' is not initialized."
                        .format(tts_action_name))
                self.tts_timer = ros_compat.create_timer(node, 0.1, self.tts_timer_cb)

        self.pub_speech = ros_compat.create_publisher(
            node, "speech_to_text", SpeechRecognitionCandidates, queue_size=1)
        self.sub_audio = ros_compat.create_subscription(node, "audio", AudioData, self.audio_cb)

    def tts_timer_cb(self, event):
        stamp = ros_compat.now_sec(self.node)
        active = False
        for tts_action in self.tts_actions:
            status = self.tts_status.get(tts_action)
            for st in (status.status_list if status else []):
                if st.status == TTS_STATUS_ACTIVE:
                    active = True
                    break
            if active:
                break
        if active:
            if not self.is_canceling:
                ros_compat.logdebug(self.node, "START CANCELLATION")
                self.is_canceling = True
                self.last_tts = None
        elif self.is_canceling:
            if self.last_tts is None:
                self.last_tts = stamp
            if stamp - self.last_tts > self.tts_tolerance:
                ros_compat.logdebug(self.node, "END CANCELLATION")
                self.is_canceling = False

    def audio_cb(self, msg):
        if self.is_canceling:
            ros_compat.loginfo(self.node, "Speech is cancelled")
            return
        data = SR.AudioData(bytes(msg.data), self.sample_rate, self.sample_width)
        try:
            ros_compat.loginfo(self.node, "Waiting for result %d" % len(data.get_raw_data()))
            result = self.recognizer.recognize_google(
                data, language=self.language)
            msg = SpeechRecognitionCandidates(
                transcript=[result],
                confidence=[1.0],
            )
            self.pub_speech.publish(msg)
        except SR.UnknownValueError as e:
            ros_compat.logerr(self.node, "Failed to recognize: %s" % str(e))
        except SR.RequestError as e:
            ros_compat.logerr(self.node, "Failed to recognize: %s" % str(e))


if __name__ == '__main__' and ros_compat is None:
    rospy.init_node("speech_to_text")
    stt = SpeechToText()
    rospy.spin()

elif __name__ == '__main__':  # with jsk_ros1_ros2_compat (ROS1/ROS2)
    node = ros_compat.init_node("speech_to_text")
    stt = SpeechToTextCompat(node)
    ros_compat.spin(node)
