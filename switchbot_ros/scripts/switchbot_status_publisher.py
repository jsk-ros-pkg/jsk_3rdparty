#!/usr/bin/env python

import os.path
from requests import ConnectionError
from switchbot_ros.switchbot import SwitchBotAPIClient
from switchbot_ros.switchbot import DeviceError, SwitchBotAPIError
from switchbot_ros.msg import Meter, PlugMini, Hub2, Bot, StripLight, MeterProCO2

try:
    from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat
    from jsk_ros1_ros2_compat.rospy_rclpy_compat import ROS_VERSION
except ImportError:
    # jsk_common < 3.0 has no jsk_ros1_ros2_compat, so this is ROS1 only:
    # forward ros_compat.xxx(node, ...) to rospy.xxx(...)
    import rospy
    ROS_VERSION = 1
    class ros_compat(object):
        Rate = staticmethod(lambda node, *args, **kwargs: rospy.Rate(*args, **kwargs))
        create_publisher = staticmethod(lambda node, *args, **kwargs: rospy.Publisher(*args, **kwargs))
        get_param = staticmethod(lambda node, *args, **kwargs: rospy.get_param(*args, **kwargs))
        init_node = staticmethod(lambda name, **kwargs: rospy.init_node(name, **kwargs))
        shutdown = staticmethod(lambda node: None)
        is_shutdown = staticmethod(lambda node, *args, **kwargs: rospy.is_shutdown(*args, **kwargs))
        loginfo = staticmethod(lambda node, *args, **kwargs: rospy.loginfo(*args, **kwargs))
        logerr = staticmethod(lambda node, *args, **kwargs: rospy.logerr(*args, **kwargs))
        logwarn_once = staticmethod(lambda node, *args, **kwargs: rospy.logwarn_once(*args, **kwargs))
        stamp_now = staticmethod(lambda node: rospy.Time.now())
if ROS_VERSION == 2:
    InterruptException = KeyboardInterrupt
else:
    import rospy
    InterruptException = rospy.ROSInterruptException


class SwitchBotStatusPublisher:
    """
    Publissh your switchbot status with ROS and SwitchBot API
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
        secret = ros_compat.get_param(node, '~secret', None)
        if secret is not None and os.path.exists(secret):
            with open(secret, 'r', encoding='utf-8') as f:
                self.secret = f.read().replace('\n', '')
        else:
            self.secret = secret

        # Initialize switchbot client
        self.bots = self.get_switchbot_client()
        self.print_apiversion()
        
        # Get parameters for publishing
        self.rate = ros_compat.get_param(node, '~rate', 0.1)
        ros_compat.loginfo(self.node, 'Rate: ' + str(self.rate))
        
        device_name = ros_compat.get_param(node, '~device_name')
        if device_name:
            self.device_name = device_name
        else:
            ros_compat.logerr(self.node, 'No Device Name')
            return
        
        self.device_type = None
        self.device_list = sorted(
            self.bots.device_list,
            key=lambda device: str(device.get('deviceName')))
        for device in self.device_list:
            device_name = str(device.get('deviceName'))
            if self.device_name == device_name:
                self.device_type = str(device.get('deviceType'))

        if self.device_type:
            ros_compat.loginfo(self.node, 'deviceName: ' + self.device_name + ' / deviceType: ' + self.device_type)
        else:
            ros_compat.logerr(self.node, 'Invalid Device Name: ' + self.device_name)
            return
        
        topic_name = '~' + self.device_name
        topic_name = topic_name.replace('-', '_')
        
        # Publisher Message Class for each device type
        if self.device_type == 'Remote':
            ros_compat.logerr(self.node, 'Device Type: "' + self.device_type + '" has no status in specifications.')
            return
        else:
            if self.device_type == 'Meter':
                self.msg_class = Meter
            elif self.device_type == 'MeterPlus':
                self.msg_class = Meter
            elif self.device_type == 'WoIOSensor':
                self.msg_class = Meter
            elif self.device_type == 'Hub 2':
                self.msg_class = Hub2
            elif self.device_type == 'Plug Mini (JP)':
                self.msg_class = PlugMini
            elif self.device_type == 'Plug Mini (US)':
                self.msg_class = PlugMini
            elif self.device_type == 'Bot':
                self.msg_class = Bot
            elif self.device_type == 'Strip Light':
                self.msg_class = StripLight
            elif self.device_type == 'MeterPro(CO2)':
                self.msg_class = MeterProCO2
            else:
                ros_compat.logerr(self.node, 'No publisher process for "' + self.device_type + '" in switchbot_status_publisher.py')
                return
            
            self.status_pub = ros_compat.create_publisher(node, topic_name, self.msg_class, queue_size=1, latch=True)
        
        ros_compat.loginfo(self.node, 'Ready: SwitchBot Status Publisher for ' + self.device_name)


    def get_switchbot_client(self):
        try:
            client = SwitchBotAPIClient(token=self.token, secret=self.secret)
            ros_compat.loginfo(self.node, 'Switchbot API Client initialized.')
            return client
        except ConnectionError:  # If the machine is not connected to the internet
            ros_compat.logwarn_once(self.node, 'Failed to connect to the switchbot server. The client would try connecting to it when subscribes the ActionGoal topic.')
            return None


    def spin(self):
        rate = ros_compat.Rate(self.node, self.rate)
        while not ros_compat.is_shutdown(self.node):
            rate.sleep()
            if ros_compat.is_shutdown(self.node):  # shut down while sleeping
                break
            if self.bots is None:
                self.bots = self.get_switchbot_client()
            
            if self.device_type == 'Remote':
                return
            else:
                status = self.get_device_status(device_name=self.device_name)
                
                if status:
                    time = ros_compat.stamp_now(self.node)
                    if self.msg_class == Meter:
                        msg = Meter()
                        msg.header.stamp = time
                        msg.temperature  = float(status['temperature'])
                        msg.humidity     = float(status['humidity'])
                        msg.battery      = float(status['battery'])
                    elif self.msg_class == Hub2:
                        msg = Hub2()
                        msg.header.stamp = time
                        msg.temperature  = float(status['temperature'])
                        msg.humidity     = float(status['humidity'])
                        msg.light_level  = status['lightLevel']
                    elif self.msg_class == PlugMini:
                        msg = PlugMini()
                        msg.header.stamp = time
                        msg.voltage      = float(status['voltage'])
                        msg.weight       = float(status['weight'])
                        msg.current      = float(status['electricCurrent'])
                        msg.minutes_day  = status['electricityOfDay']
                    elif self.msg_class == Bot:
                        msg = Bot()
                        msg.header.stamp = time
                        msg.battery      = float(status['battery'])
                        if status['power'] == 'on':
                            msg.power    = True
                        else:
                            msg.power    = False
                        msg.device_mode  = status['deviceMode']
                    elif self.msg_class == StripLight:
                        msg = StripLight()
                        msg.header.stamp = time
                        if status['power'] == 'on':
                            msg.power    = True
                        else:
                            msg.power    = False
                        msg.brightness   = status['brightness']
                        rgb_string       = status['color']
                        r, g, b = map(int, rgb_string.split(':'))
                        msg.color_r      = r
                        msg.color_g      = g
                        msg.color_b      = b
                    elif self.msg_class == MeterProCO2:
                        msg = MeterProCO2()
                        msg.header.stamp = time
                        msg.temperature  = float(status['temperature'])
                        msg.humidity     = float(status['humidity'])
                        msg.battery      = float(status['battery'])
                        msg.co2ppm       = status['CO2']
                    else:
                        return
                    
                    if msg:
                        self.status_pub.publish(msg)


    def get_device_status(self, device_name=None):
        if self.bots is None:
            return
        elif device_name:
            status = self.bots.device_status(device_name=device_name)
            return status
        else:
            return


    def print_apiversion(self):
        if self.bots is None:
            return
        
        apiversion_str = 'Using SwitchBot API ';
        apiversion_str += self.bots.api_version;
        ros_compat.loginfo(self.node, apiversion_str)


if __name__ == '__main__':
    try:
        node = ros_compat.init_node('switchbot_status_publisher')
        ssp = SwitchBotStatusPublisher(node)
        ssp.spin()
    except InterruptException:
        pass
    ros_compat.shutdown(node)
