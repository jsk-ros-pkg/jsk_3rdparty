#!/usr/bin/env python

import actionlib
import os.path
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
        # '~token' can be file path or raw characters
        token = rospy.get_param('~token')
        if os.path.exists(token):
            with open(token) as f:
                self.token = f.read().replace('\n', '')
        else:
            self.token = token
        self.bots = SwitchBotAPIClient(token=self.token)
        device_list_str = 'Switchbot device list:\n'
        for device in self.bots.device_list:
            device_list_str += 'Name: ' + str(device['deviceName'])
            device_list_str += ', Type: ' + str(device['deviceType'])
            device_list_str += '\n'
        rospy.loginfo(device_list_str)
        # Actionlib
        self._as = actionlib.SimpleActionServer(
            '~switch', SwitchBotCommandAction,
            execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

        
    def execute_cb(self, goal):
        feedback = SwitchBotCommandFeedback()
        result = SwitchBotCommandResult()
        r = rospy.Rate(1)
        success = True        
        # start executing the action
        parameter, command_type = goal.parameter, goal.command_type
        if not parameter:
            parameter = 'default'
        if not command_type:
            command_type = 'command'
        try:
            feedback.status = str(
                self.bots.control_device(
                    command=goal.command,
                    parameter=parameter,
                    command_type=command_type,
                    device_name=goal.device_name
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
    rospy.init_node('switchbot')
    server = SwitchBotAction()
    rospy.spin()
