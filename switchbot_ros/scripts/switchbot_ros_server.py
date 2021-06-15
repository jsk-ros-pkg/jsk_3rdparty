#!/usr/bin/env python

import actionlib
import rospy
from switchbot_ros.msg import SwitchBotCommandAction
from switchbot_ros.msg import SwitchBotCommandFeedback
from switchbot_ros.msg import SwitchBotCommandResult
from switchbot import SwitchBotAPIClient


class SwitchBotAction:
    """
    Control your switchbot with ROS and SwitchBot API
    """
    def __init__(self):
        # SwitchBot configs
        self.token = rospy.get_param('~token')
        self.bots = SwitchBotAPIClient(token=self.token)
        # Actionlib
        self._as = actionlib.SimpleActionServer(
            '~switch', SwitchBotCommandAction,
            execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

        
    def execute_cb(self, goal):
        self._feedback = SwitchBotCommandFeedback()
        self._result = SwitchBotCommandResult()
        r = rospy.Rate(1)
        success = True        
        # start executing the action
        parameter, command_type = goal.parameter, goal.command_type
        if not parameter:
            parameter = 'default'
        if not command_type:
            command_type = 'command'
        try:
            self._feedback.status = str(
                self.bots.control_device(
                    command=goal.command,
                    parameter=parameter,
                    command_type=command_type,
                    device_name=goal.device_name
                ))
        except Exception as e:
            rospy.logerr(str(e))
            self._feedback.status = str(e)
            success = False
        finally:
            self._as.publish_feedback(self._feedback)
            r.sleep()
            self._result.done = success
            self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('switchbot')
    server = SwitchBotAction()
    rospy.spin()
