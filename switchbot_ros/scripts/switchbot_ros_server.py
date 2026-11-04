#!/usr/bin/env python

import os.path
from requests import ConnectionError
from switchbot_ros.msg import Device
from switchbot_ros.msg import DeviceArray
from switchbot_ros.switchbot import SwitchBotAPIClient
from switchbot_ros.switchbot import DeviceError, SwitchBotAPIError

try:
    from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat
    from jsk_ros1_ros2_compat.rospy_rclpy_compat import ROS_VERSION
except ImportError:
    # jsk_common < 3.0 has no jsk_ros1_ros2_compat, so this is ROS1 only:
    # forward ros_compat.xxx(node, ...) to rospy.xxx(...)
    import actionlib
    import rospy
    ROS_VERSION = 1
    class ros_compat(object):
        Rate = staticmethod(lambda node, *args, **kwargs: rospy.Rate(*args, **kwargs))
        create_publisher = staticmethod(lambda node, *args, **kwargs: rospy.Publisher(*args, **kwargs))
        create_simple_action_server = staticmethod(lambda node, *args: actionlib.SimpleActionServer(*args, auto_start=False))
        get_param = staticmethod(lambda node, *args, **kwargs: rospy.get_param(*args, **kwargs))
        init_node = staticmethod(lambda name, **kwargs: rospy.init_node(name, **kwargs))
        spin_once = staticmethod(lambda node, timeout_sec=0.1: rospy.sleep(timeout_sec))
        shutdown = staticmethod(lambda node: None)
        is_shutdown = staticmethod(lambda node, *args, **kwargs: rospy.is_shutdown(*args, **kwargs))
        loginfo = staticmethod(lambda node, *args, **kwargs: rospy.loginfo(*args, **kwargs))
        logerr = staticmethod(lambda node, *args, **kwargs: rospy.logerr(*args, **kwargs))
        logwarn_once = staticmethod(lambda node, *args, **kwargs: rospy.logwarn_once(*args, **kwargs))
if ROS_VERSION == 2:
    from switchbot_ros.action import SwitchBotCommand as SwitchBotCommandAction
    SwitchBotCommandFeedback = SwitchBotCommandAction.Feedback
    SwitchBotCommandResult = SwitchBotCommandAction.Result
else:
    import rospy
    from switchbot_ros.msg import SwitchBotCommandAction
    from switchbot_ros.msg import SwitchBotCommandFeedback
    from switchbot_ros.msg import SwitchBotCommandResult


class SwitchBotAction:
    """
    Control your switchbot with ROS and SwitchBot API
    """
    def __init__(self, node=None):
        self.node = node
        # SwitchBot configs
        # '~token' can be file path or raw characters
        token = ros_compat.get_param(node, '~token')
        if os.path.exists(token):
            with open(token) as f:
                self.token = f.read().replace('\n', '')
        else:
            self.token = token

        # Switchbot API v1.1 needs secret key
        secret = ros_compat.get_param(node, '~secret', None )
        if secret is not None and os.path.exists(secret):
            with open(secret, 'r', encoding='utf-8') as f:
                self.secret = f.read().replace('\n', '')
        else:
            self.secret = secret

        # Initialize switchbot client
        self.bots = self.get_switchbot_client()
        self.print_apiversion()
        self.print_devices()
        self.print_scenes()
        # Actionlib
        self._as = ros_compat.create_simple_action_server(
            node, '~switch', SwitchBotCommandAction,
            self.execute_cb)
        self._as.start()
        # Topic
        self.pub = ros_compat.create_publisher(node, '~devices', DeviceArray, queue_size=1, latch=True)
        self.published = False
        ros_compat.loginfo(node, 'Ready.')

    def get_switchbot_client(self):
        try:
            client = SwitchBotAPIClient(token=self.token, secret=self.secret)
            ros_compat.loginfo(self.node, 'Switchbot API Client initialized.')
            return client
        except ConnectionError:  # If the machine is not connected to the internet
            ros_compat.logwarn_once(self.node, 'Failed to connect to the switchbot server. The client would try connecting to it when subscribes the ActionGoal topic.')
            return None

    def spin(self):
        while not ros_compat.is_shutdown(self.node):
            ros_compat.spin_once(self.node, 1.0)
            if self.bots is None:
                self.bots = self.get_switchbot_client()
            elif not self.published:
                self.publish_devices()
                self.published = True

    def print_apiversion(self):
        if self.bots is None:
            return
        
        apiversion_str = 'Using SwitchBot API ';
        apiversion_str += self.bots.api_version;
        ros_compat.loginfo(self.node, apiversion_str)
        

    def print_devices(self):
        if self.bots is None:
            return
        
        device_list_str = 'Switchbot Device List:\n'
        device_list = sorted(
            self.bots.device_list,
            key=lambda device: str(device.get('deviceName')))
        device_list_str += str(len(device_list)) + ' Item(s)\n'
        for device in device_list:
            device_list_str += 'deviceName: ' + str(device.get('deviceName'))
            device_list_str += ', deviceID: ' + str(device.get('deviceId'))
            device_list_str += ', deviceType: ' + str(device.get('deviceType'))
            device_list_str += '\n'
        ros_compat.loginfo(self.node, device_list_str)
        
        remote_list_str = 'Switchbot Remote List:\n'
        infrared_remote_list = sorted(
            self.bots.infrared_remote_list,
            key=lambda infrared_remote: str(infrared_remote.get('deviceName')))
        remote_list_str += str(len(infrared_remote_list)) + ' Item(s)\n'
        for infrared_remote in infrared_remote_list:
            remote_list_str += 'deviceName: ' + str(infrared_remote.get('deviceName'))
            remote_list_str += ', deviceID: ' + str(infrared_remote.get('deviceId'))
            remote_list_str += ', remoteType: ' + str(infrared_remote.get('remoteType'))
            remote_list_str += '\n'
        ros_compat.loginfo(self.node, remote_list_str)
        

    def print_scenes(self):
        if self.bots is None:
            return
        
        scene_list_str = 'Switchbot Scene List:\n'
        scene_list = sorted(
            self.bots.scene_list,
            key=lambda scene: str(scene.get('sceneName')))
        scene_list_str += str(len(scene_list)) + ' Item(s)\n'
        for scene in scene_list:
            scene_list_str += 'sceneName: ' + str(scene.get('sceneName'))
            scene_list_str += ', sceneID: ' + str(scene.get('sceneId'))
            scene_list_str += '\n'
        ros_compat.loginfo(self.node, scene_list_str)
        

    def publish_devices(self):
        if self.bots is None:
            return
        
        msg = DeviceArray()
        
        device_list = sorted(
            self.bots.device_list,
            key=lambda device: str(device.get('deviceName')))
        for device in device_list:
            msg_device = Device()
            msg_device.name = str(device.get('deviceName'))
            msg_device.type = str(device.get('deviceType'))
            msg.devices.append(msg_device)
        
        infrared_remote_list = sorted(
            self.bots.infrared_remote_list,
            key=lambda infrared_remote: str(infrared_remote.get('deviceName')))
        for infrared_remote in infrared_remote_list:
            msg_device = Device()
            msg_device.name = str(infrared_remote.get('deviceName'))
            msg_device.type = str(infrared_remote.get('remoteType'))
            msg.devices.append(msg_device)
        
        self.pub.publish(msg)

    def execute_cb(self, goal):
        feedback = SwitchBotCommandFeedback()
        result = SwitchBotCommandResult()
        r = ros_compat.Rate(self.node, 1)
        success = True
        # start executing the action
        parameter, command_type = goal.parameter, goal.command_type
        if not parameter:
            parameter = 'default'
        if not command_type:
            command_type = 'command'
        try:
            if not self.bots:
                self.bots = SwitchBotAPIClient(token=self.token, secret=self.secret)
            feedback.status = str(
                self.bots.control_device(
                    command=goal.command,
                    parameter=parameter,
                    command_type=command_type,
                    device_name=goal.device_name
                ))
        except (DeviceError, SwitchBotAPIError, KeyError) as e:
            ros_compat.logerr(self.node, str(e))
            feedback.status = str(e)
            success = False
        finally:
            self._as.publish_feedback(feedback)
            r.sleep()
            result.done = success
            self._as.set_succeeded(result)


if __name__ == '__main__':
    node = ros_compat.init_node('switchbot')
    server = SwitchBotAction(node)
    server.spin()
    ros_compat.shutdown(node)
