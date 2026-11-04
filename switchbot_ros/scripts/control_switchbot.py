#!/usr/bin/env python

import threading

from switchbot_ros.switchbot_ros_client import SwitchBotROSClient
from switchbot_ros.switchbot_ros_client import ros_compat

node = ros_compat.init_node('controler_node')
# SwitchBotROSClient blocks, so spin the node in another thread
threading.Thread(target=ros_compat.spin, args=(node,), daemon=True).start()
client = SwitchBotROSClient(node=node)

devices = client.get_devices()
print(devices)

client.control_device('pendant-light', 'turnOn', wait=True)

client.control_device('bot74a', 'turnOn', wait=True)
