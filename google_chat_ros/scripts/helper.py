#!/usr/bin/env python
# -*- coding: utf-8 -*-
import queue

import rospy
import actionlib

from std_msgs.msg import String
from google_chat_ros.msg import *
from dialogflow_task_executive.msg import *
from sound_play.msg import *

class GoogleChatROSHelper(object):
    """
    Helper node for google chat ROS
    """
    def __init__(self):
        # Get configuration params
        self.to_dialogflow_task_executive = rospy.get_param("~to_dialogflow_client")
        self.sound_play_jp = rospy.get_param("~debug_sound")
        self._message_sub = rospy.Subscriber("google_chat_ros/message_activity", MessageEvent, callback=self._message_cb)
        self.recent_message_event = None

    # GOOGLE CHAT
    def send_chat_client(self, goal):
        client = actionlib.SimpleActionClient('google_chat_ros/send', SendMessageAction)
        client.wait_for_server()
        client.send_goal(goal)
        client.wait_for_result()
        return client.get_result()

    def dialogflow_action_client(self, query):
        """
        :rtype: DialogTextActionResult
        """
        client = actionlib.SimpleActionClient('dialogflow_client/text_action', DialogTextAction)
        client.wait_for_server()
        goal = DialogTextActionGoal()
        goal.goal.query = query
        client.send_goal(goal.goal)
        client.wait_for_result()
        return client.get_result()

    # SOUND
    def sound_client(self, goal):
        client = actionlib.SimpleActionClient('robotsound_jp', SoundRequestAction)
        client.wait_for_server()
        client.send_goal(goal)
        client.wait_for_result()
        return client.get_result()

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
    rospy.init_node('google_chat_helper')
    node = GoogleChatROSHelper()
    rospy.spin()
