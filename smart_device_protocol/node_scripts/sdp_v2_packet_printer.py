#!/usr/bin/env python

import rospy

from smart_device_protocol.smart_device_protocol_interface import SDPInterface


def callback(src_address, frame):
    print("{} Packet from {}".format(type(frame), src_address))
    print("{}".format(frame))


if __name__ == "__main__":
    rospy.init_node("smart_device_protocol_packet_printer")
    interface = SDPInterface(callback_data=callback, callback_meta=callback)
    rospy.spin()
