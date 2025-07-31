#!/usr/bin/env python

import rospy

from smart_device_protocol.esp_now_ros_interface import ESPNOWROSInterface
from smart_device_protocol.packet_parser import parse_packet


def callback(src_address, data):
    result = parse_packet(data)
    rospy.loginfo("Packet from {} : {}".format(src_address, result))


if __name__ == "__main__":
    rospy.init_node("esp_now_packet_printer")
    interface = ESPNOWROSInterface(callback)
    rospy.spin()
