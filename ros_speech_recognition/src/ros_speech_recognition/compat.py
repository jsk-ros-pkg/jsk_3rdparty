# -*- coding: utf-8 -*-
"""`ros_compat` and `ROS_VERSION` for the scripts and modules of this
package: jsk_ros1_ros2_compat's rospy_rclpy_compat (ROS1/ROS2), or, as
jsk_common < 3.0 has no jsk_ros1_ros2_compat, the same functions forwarded
to rospy (ROS1 only)."""

try:
    from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat  # NOQA
    from jsk_ros1_ros2_compat.rospy_rclpy_compat import ROS_VERSION  # NOQA
except ImportError:
    # jsk_common < 3.0 has no jsk_ros1_ros2_compat, so this is ROS1 only:
    # forward ros_compat.xxx(node, ...) to rospy.xxx(...)
    import rospy
    ROS_VERSION = 1

    class ros_compat(object):
        init_node = staticmethod(lambda name, anonymous=False, multi_threaded=False: rospy.init_node(name, anonymous=anonymous))
        spin = staticmethod(lambda node: rospy.spin())
        get_param = staticmethod(lambda node, *args, **kwargs: rospy.get_param(*args, **kwargs))
        has_param = staticmethod(lambda node, name: rospy.has_param(name))
        loginfo = staticmethod(lambda node, msg: rospy.loginfo(msg))
        logwarn = staticmethod(lambda node, msg: rospy.logwarn(msg))
        logerr = staticmethod(lambda node, msg: rospy.logerr(msg))
        logdebug = staticmethod(lambda node, msg: rospy.logdebug(msg))
        create_publisher = staticmethod(lambda node, topic, msg_type, queue_size=1, latch=False: rospy.Publisher(topic, msg_type, queue_size=queue_size, latch=latch))
        create_subscription = staticmethod(lambda node, topic, msg_type, callback, queue_size=None, latch=False: rospy.Subscriber(topic, msg_type, callback, queue_size=queue_size))
        create_service = staticmethod(lambda node, name, srv_type, callback: rospy.Service(name, srv_type, lambda req: callback(req, srv_type._response_class())))
        create_timer = staticmethod(lambda node, period, callback: rospy.Timer(rospy.Duration(period), callback))
        now_sec = staticmethod(lambda node: rospy.Time.now().to_sec())
        sleep = staticmethod(lambda node, seconds: rospy.sleep(seconds))
        is_shutdown = staticmethod(lambda node: rospy.is_shutdown())
        on_shutdown = staticmethod(lambda node, callback: rospy.on_shutdown(callback))

        @staticmethod
        def create_reconfigure_server(node, config_type, callback, defaults=None):
            from dynamic_reconfigure.server import Server
            return Server(config_type, callback)

        @staticmethod
        def create_simple_action_client(node, name, action_type):
            import actionlib

            # timeouts in seconds, as jsk_ros1_ros2_compat's
            class SimpleActionClient(actionlib.SimpleActionClient):
                def wait_for_server(self, timeout=None):
                    return actionlib.SimpleActionClient.wait_for_server(
                        self, rospy.Duration(timeout or 0))

                def wait_for_result(self, timeout=None):
                    return actionlib.SimpleActionClient.wait_for_result(
                        self, rospy.Duration(timeout or 0))

                def send_goal_and_wait(self, goal, timeout=None):
                    self.send_goal(goal)
                    return self.wait_for_result(timeout)
            return SimpleActionClient(name, action_type)
