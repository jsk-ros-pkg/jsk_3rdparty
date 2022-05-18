#!/usr/bin/env python

import actionlib
import os.path
from requests import ConnectionError
import rospy
from switchbot_ros.msg import SwitchBotCommandAction
from switchbot_ros.msg import SwitchBotCommandFeedback
from switchbot_ros.msg import SwitchBotCommandResult
from switchbot_ros.msg import Device
from switchbot_ros.msg import DeviceArray
from switchbot_ros.switchbot import SwitchBotAPIClient
from switchbot_ros.switchbot import DeviceError, SwitchBotAPIError


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
        # Initialize switchbot client
        self.bots = self.get_switchbot_client()
        self.print_devices()
        # Actionlib
        self._as = actionlib.SimpleActionServer(
            '~switch', SwitchBotCommandAction,
            execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        # Topic
        self.pub = rospy.Publisher('~devices', DeviceArray, queue_size=1, latched=True)
        self.published = False

    def get_switchbot_client(self):
        try:
            return SwitchBotAPIClient(token=self.token)
        except ConnectionError:  # If the machine is not connected to the internet
            rospy.logwarn_once('Failed to connect to the switchbot server. The client would try connecting to it when subscribes the ActionGoal topic.')
            return None

    def spin(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()
            if self.bots is None:
                self.bots = self.get_switchbot_client()
            elif not self.published:
                self.publish_devices()
                self.published = True

    def print_devices(self):
        if self.bots is None:
            return
        device_list_str = 'Switchbot device list:\n'
        device_list = sorted(
            self.bots.device_list,
            key=lambda device: str(device['deviceName']))
        for device in device_list:
            device_list_str += 'Name: ' + str(device['deviceName'])
            device_list_str += ', Type: ' + str(device['deviceType'])
            device_list_str += '\n'
        rospy.loginfo(device_list_str)

    def publish_devices(self):
        if self.bots is None:
            return
        msg = DeviceArray()
        device_list = sorted(
            self.bots.device_list,
            key=lambda device: str(device['deviceName']))
        for device in device_list:
            msg_device = Device()
            msg_device.name = str(device['deviceName'])
            msg_device.type = str(device['deviceType'])
            msg.devices.append(msg_device)
        self.pub.publish(msg)

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
            if not self.bots:
                self.bots = SwitchBotAPIClient(token=self.token)
            feedback.status = str(
                self.bots.control_device(
                    command=goal.command,
                    parameter=parameter,
                    command_type=command_type,
                    device_name=goal.device_name
                ))
        except (DeviceError, SwitchBotAPIError, KeyError) as e:
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
    server.spin()
