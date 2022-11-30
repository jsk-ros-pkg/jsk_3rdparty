#!/usr/bin/env python

import rospy
import nfc_ros
from nfc_ros.nfc_ros_node import NFCROSNode


if __name__ == '__main__':

    rospy.init_node('nfc_ros')
    node = NFCROSNode()
    node.spin()
