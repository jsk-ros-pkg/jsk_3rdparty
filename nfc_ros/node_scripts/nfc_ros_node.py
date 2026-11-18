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
        errors = []

        def read_cards():
            # an exception of reading the cards would end only this thread,
            # and leave the node running without reading: shut the node down
            # and exit with an error, as on ROS1
            try:
                nfc_ros_node.spin()
            except Exception as e:
                errors.append(e)
                ros_compat.logerr(node, 'Failed to read the NFC cards: {}'.format(e))
                ros_compat.signal_shutdown(node, 'failed to read the NFC cards')
        thread = threading.Thread(target=read_cards)
        thread.daemon = True
        thread.start()
        ros_compat.spin(node)
        thread.join(timeout=1.0)
        if errors:
            sys.exit(1)
