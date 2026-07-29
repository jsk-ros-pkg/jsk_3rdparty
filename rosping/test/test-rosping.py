#!/usr/bin/env python
import os
import stat
import unittest

import rospy
import roslib.packages
from std_msgs.msg import Float64

PKG = 'rosping'
NAME = 'test_rosping'


class TestRosping(unittest.TestCase):

    def test_ping_delay(self):
        paths = roslib.packages.find_node('rosping', 'rosping')
        self.assertTrue(paths, "could not locate the rosping executable")
        binary = paths[0]

        # raw ICMP sockets require CAP_NET_RAW, which rosping gets either by
        # running as root or via a setuid-root install (see CMakeLists.txt).
        # Skip rather than fail when neither is available, since that's an
        # environment limitation, not a regression in rosping itself.
        privileged = os.geteuid() == 0 or bool(os.stat(binary).st_mode & stat.S_ISUID)
        if not privileged:
            raise unittest.SkipTest(
                "%s is not setuid-root and this test is not running as root; "
                "raw ICMP sockets require elevated privileges. Run "
                "'sudo chown root:root %s && sudo chmod 4755 %s' once to enable "
                "this test." % (binary, binary, binary))

        rospy.init_node(NAME)
        received = []
        rospy.Subscriber("/ping/delay", Float64, received.append)

        timeout = rospy.Time.now() + rospy.Duration(10.0)
        while not rospy.is_shutdown() and rospy.Time.now() < timeout and not received:
            rospy.sleep(0.1)

        self.assertTrue(received, "no messages received on /ping/delay before timeout")


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestRosping)
