import rospy
import actionlib
from switchbot_ros.msg import SwitchBotCommandAction
from switchbot_ros.msg import SwitchBotCommandGoal
from switchbot_ros.msg import DeviceArray


class SwitchBotROSClient(object):

    def __init__(self,
                 actionname='switchbot_ros/switch',
                 topicname='switchbot_ros/devices',
                 timeout=5):

        self.actionname = actionname
        self.topicname = topicname
        self.action_client = actionlib.SimpleActionClient(
                actionname,
                SwitchBotCommandAction
                )
        rospy.loginfo("Waiting for action server to start. (timeout: " + str(timeout) + "[sec])")
        self.action_client.wait_for_server(timeout=rospy.Duration(timeout,0))

    def get_devices(self, timeout=None):

        return rospy.wait_for_message(
                self.topicname,
                DeviceArray,
                timeout=timeout
                )

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
