#!/usr/bin/env python

import rospy
from switchbot_ros.switchbot_ros_client import SwitchBotROSClient

rospy.init_node('controler_node')
client = SwitchBotROSClient()

devices = client.get_devices()
print(devices)

client.control_device('pendant-light', 'turnOn', wait=True)

client.control_device('bot74a', 'turnOn', wait=True)

