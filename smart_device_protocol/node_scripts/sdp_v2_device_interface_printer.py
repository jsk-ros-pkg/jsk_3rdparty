#!/usr/bin/env python

import rospy

from smart_device_protocol.smart_device_protocol_interface import DeviceDictSDPInterface


if __name__ == "__main__":
    rospy.init_node("smart_device_protocol_device_interface_printer")
    sdp_interface = DeviceDictSDPInterface()
    while not rospy.is_shutdown():
        print("Current Device Interfaces:")
        for src_address, device_interface in sdp_interface.device_interfaces.items():
            print("Address: {}, Device: {}".format(src_address, device_interface))
        rospy.sleep(1.0)
