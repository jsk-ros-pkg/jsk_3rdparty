#!/usr/bin/env python

import unittest
import rospy
import rostest
from smart_device_protocol.smart_device_protocol_interface import (
    DeviceDictSDPInterfaceWithInterfaceCallback,
)


class TestCase(unittest.TestCase):
    def callback_test(self, src_address, content):
        print("{}: {}".format(src_address, content))
        self.called = True

    def test_sdp_interface(self):
        self.called = False
        rospy.init_node("test_sdp_interface")
        sdp_interface = DeviceDictSDPInterfaceWithInterfaceCallback()
        sdp_interface.register_callback(("Light status", "?"), self.callback_test)
        rospy.sleep(5)
        self.assertTrue(len(sdp_interface.device_interfaces) > 0)
        self.assertTrue(self.called)
        sdp_interface.unregister_callback(("Light status", "?"))
        self.called = False
        rospy.sleep(5)
        self.assertFalse(self.called)


if __name__ == "__main__":
    rostest.rosrun("smart_device_protocol", "test_sdp_interface", TestCase)
