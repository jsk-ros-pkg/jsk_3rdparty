#!/usr/bin/env python
import actionlib
from google_chat_ros.google_chat import GoogleChatRESTClient
from google_chat_ros.msg import SendMessageAction
from google_chat_ros.msg import SendMessageFeedback
from google_chat_ros.msg import SendMessageResult
import rospy


class GoogleChatActionServer:
    """
    Send request to Google Chat REST API via ROS
    """
    def __init__(self):
        keyfile = rospy.get_param('~keyfile')
        self._client = GoogleChatRESTClient(keyfile)
        # Start google chat authentication and service
        rospy.loginfo("Starting Google Chat service...")
        try:
            self._client.build_service()
            rospy.loginfo("Succeeded in starting Google Chat service")
        except Exception as e:
            rospy.logwarn("Failed to start Google Chat service")
            rospy.logerr(e)
        # ActionLib
        self._as = actionlib.SimpleActionServer(
            '~send', SendMessageAction,
            execute_cb=self.execute_cb, auto_start=False
        )
        self._as.start()

    def execute_cb(self, goal):
        feedback = SendMessageFeedback()
        result = SendMessageResult()
        r = rospy.Rate(1)
        success = True
        # start executing the action
        space = goal.space
        message_type = goal.message_type
        content = goal.content
        try:
            # establish the service
            self._client.build_service()
            if message_type == 'text':
                rospy.loginfo("Send text type message")
                feedback.status = str(
                    self._client.send_text(
                        space=space,
                        text=content
                    ))
            elif message_type == 'card':
                rospy.loginfo("Send card type message")
                feedback.status = str(
                    self._client.send_card(
                        space=space,
                        content=content
                    ))
        except Exception as e:
            rospy.logerr(str(e))
            feedback.status = str(e)
            success = False
        finally:
            self._as.publish_feedback(feedback)
            r.sleep()
            result.done = success
            self._as.set_succeeded(result)


if __name__ == '__main__':
    rospy.init_node('google_chat')
    server = GoogleChatActionServer()
    rospy.spin()
