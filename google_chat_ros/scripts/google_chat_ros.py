#!/usr/bin/env python
import actionlib
from google_chat_ros.client import GoogleChatClient
from google_chat_ros.msg import GoogleChatAction
from google_chat_ros.msg import GoogleChatFeedback
from google_chat_ros.msg import GoogleChatResult
import os.path
import rospy

class GoogleChatActionServer:
    """
    Send request to Google Chat REST API via ROS
    """
    def __init__(self):
        keyfile = rospy.get_param('~keyfile')
        self._client = GoogleChatClient(keyfile)
        try:
            self.build_service()
        except Exception as e:
            rospy.logerr(e)
        # ActionLib
        self._as = actionlib.SimpleActionServer(
            '~googlechat', GoogleChatAction,
            execute_cb=self.execute_cb, auto_start=False
        )
        self._as.start()

    def execute_cb(self, goal):
        pass
