#!/usr/bin/env python

import sys

from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat
import nfc_ros
from nfc_ros.nfc_ros_node import NFCROSNode


if __name__ == '__main__':

    node = ros_compat.init_node('nfc_ros')
    try:
        nfc_ros_node = NFCROSNode(node)
    except IOError as e:
        # nfc.ContactlessFrontend() raises ENODEV without a reader, or when
        # it is not accessible (see README.md for the udev rule)
        ros_compat.logerr(node, 'Failed to open the NFC reader: {}'.format(e))
        sys.exit(1)
    if ros_compat.ROS_VERSION == 1:
        nfc_ros_node.spin()
    else:
        # rclpy's signal handlers shut the context down only out of a
        # spinning executor (and nfcpy's clf.connect() swallows the
        # KeyboardInterrupt), so read the cards in a thread, which returns
        # when ros_compat.spin() is shut down by SIGINT/SIGTERM
        import threading
        thread = threading.Thread(target=nfc_ros_node.spin)
        thread.daemon = True
        thread.start()
        ros_compat.spin(node)
        thread.join(timeout=1.0)
