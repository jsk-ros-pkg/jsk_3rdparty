#!/usr/bin/env python

import rospy
import actionlib
from switchbot_ros.msg import SwitchBotCommandAction, SwitchBotCommandFeedback, SwitchBotCommandResult
from switchbot import SwitchBotAPIClient


class SwitchBotAction:
    """
    Control your switchbot with ROS and SwitchBot API
    """
    _feedback = SwitchBotCommandFeedback()
    _result = SwitchBotCommandResult()
    
    def __init__(self, name):
        # SwitchBot configs
        self.token = rospy.get_param('~token')
        self.bots = SwitchBotAPIClient(token=self.token)
        # Actionlib
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, SwitchBotCommandAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

        
    def execute_cb(self, goal):
        r = rospy.Rate(1)
        success = True
        
        # start executing the action
        parameter, commandType = goal.parameter, goal.commandType
        if not parameter:
            parameter = 'default'
        if not commandType:
            commandType = 'command'
        try:
            self._feedback.status = str(self.bots.control_device(command=goal.command, parameter=parameter, commandType=commandType, deviceName=goal.deviceName))
        except Exception as e:
            self._feedback.status = str(e)
            success = False
        finally:
            self._as.publish_feedback(self._feedback)
            r.sleep()
            self._result.done = success
            self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('switchbot')
    server = SwitchBotAction(rospy.get_name())
    rospy.spin()
