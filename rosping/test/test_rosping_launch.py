# ROS2 counterpart of test-rosping.test
import os
import stat
import time
import unittest

from ament_index_python.packages import get_package_prefix
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.util
import pytest

import rclpy
from std_msgs.msg import Float64


@pytest.mark.launch_test
def generate_test_description():
    rosping_node = launch_ros.actions.Node(
        package='rosping',
        executable='rosping',
        arguments=['127.0.0.1'],
        parameters=[{'rate': 0.5}],  # 2hz
        output='screen',
    )
    return launch.LaunchDescription([
        rosping_node,
        # rosping exits at once without the privileges, keep the launch
        # running for the tests (which are skipped then)
        launch_testing.util.KeepAliveProc(),
        launch_testing.actions.ReadyToTest(),
    ]), {'rosping_node': rosping_node}


class TestRosping(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_rosping')

    def tearDown(self):
        self.node.destroy_node()

    def test_ping_delay(self):
        binary = os.path.join(get_package_prefix('rosping'), 'lib', 'rosping', 'rosping')
        self.assertTrue(os.path.exists(binary), "could not locate the rosping executable")

        # raw ICMP sockets require CAP_NET_RAW, which rosping gets either by
        # running as root or via a setuid-root install (see CMakeLists.txt).
        # Skip rather than fail when neither is available, since that's an
        # environment limitation, not a regression in rosping itself.
        st = os.stat(binary)
        privileged = os.geteuid() == 0 or (st.st_uid == 0 and bool(st.st_mode & stat.S_ISUID))
        if not privileged:
            raise unittest.SkipTest(
                "%s is not setuid-root and this test is not running as root; "
                "raw ICMP sockets require elevated privileges. Run "
                "'sudo chown root:root %s && sudo chmod 4755 %s' once to enable "
                "this test." % (binary, binary, binary))

        received = []
        self.node.create_subscription(Float64, '/ping/delay', received.append, 10)

        end_time = time.time() + 10.0
        while time.time() < end_time and not [m for m in received if m.data >= 0]:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertTrue(received, "no messages received on /ping/delay before timeout")
        # 127.0.0.1 replies, unlike 8.8.8.8 of the ROS1 test without the
        # network, so the delay (-1 on timeout) is measured
        self.assertTrue([m for m in received if m.data >= 0],
                        "no reply from 127.0.0.1: %s" % [m.data for m in received])


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self, proc_info, rosping_node):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=rosping_node,
            allowable_exit_codes=[0, -2, -15])
