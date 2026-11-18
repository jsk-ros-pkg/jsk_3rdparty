#!/usr/bin/env python

from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat
import nfc_ros
from nfc_ros.nfc_ros_node import NFCROSNode


if __name__ == '__main__':

    node = ros_compat.init_node('nfc_ros')
    nfc_ros_node = NFCROSNode(node)
    nfc_ros_node.spin()
