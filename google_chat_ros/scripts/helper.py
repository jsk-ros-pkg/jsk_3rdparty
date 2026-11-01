#!/usr/bin/env python
# -*- coding: utf-8 -*-
import queue
import threading

from std_msgs.msg import String

try:
    from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat
    from jsk_ros1_ros2_compat.rospy_rclpy_compat import ROS_VERSION
except ImportError:
    # jsk_common < 3.0 has no jsk_ros1_ros2_compat, so this is ROS1 only:
    # forward ros_compat.xxx(node, ...) to rospy.xxx(...)
    import actionlib
    import rospy
    ROS_VERSION = 1
    class ros_compat(object):
        create_simple_action_client = staticmethod(lambda node, name, action_type: actionlib.SimpleActionClient(name, action_type))
        create_subscription = staticmethod(lambda node, *args, **kwargs: rospy.Subscriber(*args, **kwargs))
        get_param = staticmethod(lambda node, *args, **kwargs: rospy.get_param(*args, **kwargs))
        init_node = staticmethod(lambda name, **kwargs: rospy.init_node(name, **kwargs))
        spin = staticmethod(lambda node: rospy.spin())
        logwarn = staticmethod(lambda node, *args, **kwargs: rospy.logwarn(*args, **kwargs))
if ROS_VERSION == 2:
    from google_chat_ros.action import SendMessage as SendMessageAction
    from google_chat_ros.msg import MessageEvent
    from sound_play_msgs.action import SoundRequest as SoundRequestAction
    SendMessageGoal = SendMessageAction.Goal
    SoundRequestGoal = SoundRequestAction.Goal
    try:
        from dialogflow_task_executive.action import DialogText as DialogTextAction
        DialogTextGoal = DialogTextAction.Goal
    except ImportError:
        # dialogflow_task_executive is not ported to ROS2 yet
        DialogTextAction = None
else:
    import rospy
    from google_chat_ros.msg import *
    from dialogflow_task_executive.msg import *
    from sound_play.msg import *

class GoogleChatROSHelper(object):
    """
    Helper node for google chat ROS
    """
    def __init__(self, node=None):
        self.node = node
        # Get configuration params
        self.to_dialogflow_task_executive = ros_compat.get_param(node, "~to_dialogflow_client")
        self.sound_play_jp = ros_compat.get_param(node, "~debug_sound")
        if self.to_dialogflow_task_executive and DialogTextAction is None:
            ros_compat.logwarn(node, "to_dialogflow_client is ignored: dialogflow_task_executive is not available")
            self.to_dialogflow_task_executive = False
        # the action clients below block until the result, which must not
        # happen in a ROS2 executor callback, so _message_cb() runs in a
        # worker thread (process_messages())
        self.message_queue = queue.Queue()
        self._message_sub = ros_compat.create_subscription(node, "google_chat_ros/message_activity", MessageEvent, self.message_queue.put)
        self.recent_message_event = None
        self._clients = {}

    def _client(self, name, action_type):
        if name not in self._clients:
            self._clients[name] = ros_compat.create_simple_action_client(self.node, name, action_type)
        return self._clients[name]

    # GOOGLE CHAT
    def send_chat_client(self, goal):
        client = self._client('google_chat_ros/send', SendMessageAction)
        client.wait_for_server()
        client.send_goal(goal)
        client.wait_for_result()
        return client.get_result()

    def dialogflow_action_client(self, query):
        """
        :rtype: DialogTextActionResult
        """
        client = self._client('dialogflow_client/text_action', DialogTextAction)
        client.wait_for_server()
        goal = DialogTextGoal()
        goal.query = query
        client.send_goal(goal)
        client.wait_for_result()
        return client.get_result()

    # SOUND
    def sound_client(self, goal):
        client = self._client('robotsound_jp', SoundRequestAction)
        client.wait_for_server()
        client.send_goal(goal)
        client.wait_for_result()
        return client.get_result()

    def process_messages(self):
        while True:
            self._message_cb(self.message_queue.get())

    def _message_cb(self, data):
        """
        Callback function for subscribing MessageEvent.msg
        """
        sender_id = data.message.sender.name
        sender_name = data.message.sender.display_name
        space = data.space.name
        thread_name = data.message.thread_name
        text = data.message.argument_text
        if self.to_dialogflow_task_executive:
            chat_goal = SendMessageGoal()
            chat_goal.space = space
            chat_goal.thread_name = thread_name
            dialogflow_res = self.dialogflow_action_client(text)
            content = "<{}> {}".format(sender_id, dialogflow_res.response.response)
            chat_goal.text = content
            self.send_chat_client(chat_goal)
        if self.sound_play_jp:
            sound_goal = SoundRequestGoal()
            sound_goal.sound_request.sound = sound_goal.sound_request.SAY
            sound_goal.sound_request.command = sound_goal.sound_request.PLAY_ONCE
            sound_goal.sound_request.volume = 1.0
            sound_goal.sound_request.arg = "{}さんから，{}というメッセージを受信しました".format(sender_name, text)
            self.sound_client(sound_goal)

if __name__ == '__main__':
    node = ros_compat.init_node('google_chat_helper')
    helper = GoogleChatROSHelper(node)
    worker = threading.Thread(target=helper.process_messages)
    worker.daemon = True
    worker.start()
    # spin in the main thread: rclpy handles SIGINT/SIGTERM only there
    ros_compat.spin(node)
