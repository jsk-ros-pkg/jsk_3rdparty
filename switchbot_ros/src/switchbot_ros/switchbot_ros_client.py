import threading

try:
    from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat
    from jsk_ros1_ros2_compat.rospy_rclpy_compat import ROS_VERSION
except ImportError:
    # jsk_common < 3.0 has no jsk_ros1_ros2_compat, so this is ROS1 only:
    # forward ros_compat.xxx(node, ...) to rospy/actionlib
    import actionlib
    import rospy
    ROS_VERSION = 1

    class _SimpleActionClient(actionlib.SimpleActionClient):
        # timeouts in seconds, as ros_compat.create_simple_action_client()
        def wait_for_server(self, timeout=None):
            return actionlib.SimpleActionClient.wait_for_server(self, rospy.Duration(timeout or 0))

        def wait_for_result(self, timeout=None):
            return actionlib.SimpleActionClient.wait_for_result(self, rospy.Duration(timeout or 0))

    class ros_compat(object):
        create_simple_action_client = staticmethod(lambda node, *args: _SimpleActionClient(*args))
        create_subscription = staticmethod(lambda node, topic, msg_type, callback, queue_size=None, latch=False: rospy.Subscriber(topic, msg_type, callback, queue_size=queue_size))
        init_node = staticmethod(lambda name, **kwargs: rospy.init_node(name, **kwargs))
        spin = staticmethod(lambda node: rospy.spin())
        loginfo = staticmethod(lambda node, *args, **kwargs: rospy.loginfo(*args, **kwargs))
if ROS_VERSION == 2:
    from switchbot_ros.action import SwitchBotCommand as SwitchBotCommandAction
    SwitchBotCommandGoal = SwitchBotCommandAction.Goal
else:
    import rospy
    from switchbot_ros.msg import SwitchBotCommandAction
    from switchbot_ros.msg import SwitchBotCommandGoal
from switchbot_ros.msg import DeviceArray


class SwitchBotROSClient(object):
    """On ROS2, `node` must be spun by another thread (e.g. a
    MultiThreadedExecutor), since the calls below block."""

    def __init__(self,
                 actionname='switchbot_ros/switch',
                 topicname='switchbot_ros/devices',
                 timeout=5,
                 node=None):

        self.node = node
        self.actionname = actionname
        self.topicname = topicname
        self.action_client = ros_compat.create_simple_action_client(
                node,
                actionname,
                SwitchBotCommandAction
                )
        ros_compat.loginfo(node, "Waiting for action server to start. (timeout: " + str(timeout) + "[sec])")
        self.action_client.wait_for_server(timeout)

    def get_devices(self, timeout=None):

        # ~devices is latched by switchbot_ros_server.py
        received = []
        event = threading.Event()
        sub = ros_compat.create_subscription(
            self.node, self.topicname, DeviceArray,
            lambda msg: (received.append(msg), event.set()), latch=True)
        event.wait(timeout)
        if ROS_VERSION == 2:
            self.node.destroy_subscription(sub)
        else:
            sub.unregister()
        if not received:
            raise RuntimeError(
                'timeout exceeded while waiting for message on topic '
                + self.topicname)
        return received[0]

    def control_device(self,
                       device_name,
                       command,
                       parameter='',
                       command_type='',
                       wait=False
                       ):

        goal = SwitchBotCommandGoal()
        goal.device_name = device_name
        goal.command = command
        goal.parameter = parameter
        goal.command_type = command_type
        self.action_client.send_goal(goal)
        if wait:
            self.action_client.wait_for_result()
            return self.action_client.get_result()
