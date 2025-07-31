#!/usr/bin/env python

import rospy

from smart_device_protocol.esp_now_ros_interface import ESPNOWROSInterface
from smart_device_protocol.packet_generator import create_test_packet

if __name__ == "__main__":
    rospy.init_node("broadcast_test_packet")

    interface = ESPNOWROSInterface()

    rate = rospy.Rate(0.2)
    count = 0
    while not rospy.is_shutdown():
        rate.sleep()
        data = create_test_packet(num_int=count, string="count: {}".format(count))
        interface.send((255, 255, 255, 255, 255, 255), data)
        rospy.loginfo("send data: {}".format(data))
        count += 1
