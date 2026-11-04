#!/usr/bin/env python
try:
    from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat
except ImportError:
    ros_compat = None  # jsk_common < 3.0 (ROS1 only): use the original rospy code

if ros_compat is None:
    try: # catkin does not requires load_manifest
        import rospatlite
    except:
        import roslib; roslib.load_manifest('rospatlite')

    import rospy
from std_msgs.msg import Int8

from patlite import Patlite
from patlite import PatliteState
import socket

class PatliteNode(object):
    def set_state(self, target, state):
        self.pstate.set_from_int(target, state)
        self.send_flag = True

    def callback_r(self, msg):
        self.set_state(self.pstate.Target.LIGHT_RED, msg.data)

    def callback_y(self, msg):
        self.set_state(self.pstate.Target.LIGHT_YELLOW, msg.data)

    def callback_g(self, msg):
        self.set_state(self.pstate.Target.LIGHT_GREEN, msg.data)

    def callback_b(self, msg):
        self.set_state(self.pstate.Target.LIGHT_BLUE, msg.data)

    def callback_w(self, msg):
        self.set_state(self.pstate.Target.LIGHT_WHITE, msg.data)

    def callback_buzz(self, msg):
        self.set_state(self.pstate.Target.BUZZER, msg.data)

    def run(self):
        rospy.init_node('patlite_node')
        patlite_host = rospy.get_param('~host', "10.68.0.10")
        patlite_port = rospy.get_param('~port', 10000)
        timeout = rospy.get_param('~timeout', 0.5)
        self.p = Patlite(patlite_host, port=patlite_port, timeout=timeout)
        self.pstate = PatliteState()
        self.send_flag = False

        rospy.Subscriber("~set/red", Int8, self.callback_r)
        rospy.Subscriber("~set/yellow", Int8, self.callback_y)
        rospy.Subscriber("~set/green", Int8, self.callback_g)
        rospy.Subscriber("~set/blue", Int8, self.callback_b)
        rospy.Subscriber("~set/white", Int8, self.callback_w)
        rospy.Subscriber("~set/buzzer", Int8, self.callback_buzz)
        rospy.loginfo("[patlite_node] started")

        while not rospy.is_shutdown():
            if self.send_flag:
                self.send_flag = False
                try:
                    with self.p:
                        self.p.write(self.pstate)
                        self.pstate.clear()
                except socket.timeout:
                    rospy.logerr("[patlite_node] Error: socket timeout (%s:%d)",
                            patlite_host, patlite_port)
                except socket.error as e:
                    rospy.logerr("[patlite_node] Error: %s (%s:%d)",
                            e, patlite_host, patlite_port)
            rospy.sleep(0.1)
        rospy.loginfo("[patlite_node] shutdown")
        return

    def run_compat(self, node):
        """run() written with jsk_ros1_ros2_compat, for both ROS1 and ROS2"""
        patlite_host = ros_compat.get_param(node, '~host', "10.68.0.10")
        patlite_port = ros_compat.get_param(node, '~port', 10000)
        timeout = ros_compat.get_param(node, '~timeout', 0.5)
        self.p = Patlite(patlite_host, port=patlite_port, timeout=timeout)
        self.pstate = PatliteState()
        self.send_flag = False

        ros_compat.create_subscription(node, "~set/red", Int8, self.callback_r)
        ros_compat.create_subscription(node, "~set/yellow", Int8, self.callback_y)
        ros_compat.create_subscription(node, "~set/green", Int8, self.callback_g)
        ros_compat.create_subscription(node, "~set/blue", Int8, self.callback_b)
        ros_compat.create_subscription(node, "~set/white", Int8, self.callback_w)
        ros_compat.create_subscription(node, "~set/buzzer", Int8, self.callback_buzz)
        ros_compat.loginfo(node, "[patlite_node] started")

        while not ros_compat.is_shutdown(node):
            if self.send_flag:
                self.send_flag = False
                try:
                    with self.p:
                        self.p.write(self.pstate)
                        self.pstate.clear()
                except socket.timeout:
                    ros_compat.logerr(node, "[patlite_node] Error: socket timeout (%s:%d)" % (
                            patlite_host, patlite_port))
                except socket.error as e:
                    ros_compat.logerr(node, "[patlite_node] Error: %s (%s:%d)" % (
                            e, patlite_host, patlite_port))
            ros_compat.spin_once(node, 0.1)
        ros_compat.loginfo(node, "[patlite_node] shutdown")
        return

if __name__ == '__main__' and ros_compat is None:
    try:
        pn = PatliteNode()
        pn.run()
    except rospy.ROSInterruptException:
        pass

elif __name__ == '__main__':  # with jsk_ros1_ros2_compat (ROS1/ROS2)
    node = ros_compat.init_node('patlite_node')
    pn = PatliteNode()
    pn.run_compat(node)
    ros_compat.shutdown(node)
