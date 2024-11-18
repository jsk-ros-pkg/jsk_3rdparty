#!/usr/bin/env python

import os.path
from requests import ConnectionError
import rospy
from switchbot_ros.switchbot import SwitchBotAPIClient
from switchbot_ros.switchbot import DeviceError, SwitchBotAPIError
from switchbot_ros.msg import Meter, PlugMini, Hub2, Bot, StripLight, MeterProCO2


class SwitchBotStatusPublisher:
    """
    Publissh your switchbot status with ROS and SwitchBot API
    """
    def __init__(self):
        # SwitchBot configs
        # '~token' can be file path or raw characters
        token = rospy.get_param('~token')
        if os.path.exists(token):
            with open(token) as f:
                self.token = f.read().replace('\n', '')
        else:
            self.token = token

        # Switchbot API v1.1 needs secret key
        secret = rospy.get_param('~secret', None)
        if secret is not None and os.path.exists(secret):
            with open(secret, 'r', encoding='utf-8') as f:
                self.secret = f.read().replace('\n', '')
        else:
            self.secret = secret

        # Initialize switchbot client
        self.bots = self.get_switchbot_client()
        self.print_apiversion()
        
        # Get parameters for publishing
        self.rate = rospy.get_param('~rate', 0.1)
        rospy.loginfo('Rate: ' + str(self.rate))
        
        device_name = rospy.get_param('~device_name')
        if device_name:
            self.device_name = device_name
        else:
            rospy.logerr('No Device Name')
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
            rospy.loginfo('deviceName: ' + self.device_name + ' / deviceType: ' + self.device_type)
        else:
            rospy.logerr('Invalid Device Name: ' + self.device_name)
            return
        
        topic_name = '~' + self.device_name
        topic_name = topic_name.replace('-', '_')
        
        # Publisher Message Class for each device type
        if self.device_type == 'Remote':
            rospy.logerr('Device Type: "' + self.device_type + '" has no status in specifications.')
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
                rospy.logerr('No publisher process for "' + self.device_type + '" in switchbot_status_publisher.py')
                return
            
            self.status_pub = rospy.Publisher(topic_name, self.msg_class, queue_size=1, latch=True)
        
        rospy.loginfo('Ready: SwitchBot Status Publisher for ' + self.device_name)


    def get_switchbot_client(self):
        try:
            client = SwitchBotAPIClient(token=self.token, secret=self.secret)
            rospy.loginfo('Switchbot API Client initialized.')
            return client
        except ConnectionError:  # If the machine is not connected to the internet
            rospy.logwarn_once('Failed to connect to the switchbot server. The client would try connecting to it when subscribes the ActionGoal topic.')
            return None


    def spin(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()
            if self.bots is None:
                self.bots = self.get_switchbot_client()
            
            if self.device_type == 'Remote':
                return
            else:
                status = self.get_device_status(device_name=self.device_name)
                
                if status:
                    time = rospy.get_rostime()
                    if self.msg_class == Meter:
                        msg = Meter()
                        msg.header.stamp = time
                        msg.temperature  = status['temperature']
                        msg.humidity     = status['humidity']
                        msg.battery      = status['battery']
                    elif self.msg_class == Hub2:
                        msg = Hub2()
                        msg.header.stamp = time
                        msg.temperature  = status['temperature']
                        msg.humidity     = status['humidity']
                        msg.light_level  = status['lightLevel']
                    elif self.msg_class == PlugMini:
                        msg = PlugMini()
                        msg.header.stamp = time
                        msg.voltage      = status['voltage']
                        msg.weight       = status['weight']
                        msg.current      = status['electricCurrent']
                        msg.minutes_day  = status['electricityOfDay']
                    elif self.msg_class == Bot:
                        msg = Bot()
                        msg.header.stamp = time
                        msg.battery      = status['battery']
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
                        msg.temperature  = status['temperature']
                        msg.humidity     = status['humidity']
                        msg.battery      = status['battery']
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
        rospy.loginfo(apiversion_str)


if __name__ == '__main__':
    try:
        rospy.init_node('switchbot_status_publisher')
        ssp = SwitchBotStatusPublisher()
        ssp.spin()
    except rospy.ROSInterruptException:
        pass
