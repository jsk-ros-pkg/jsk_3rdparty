#!/usr/bin/env python
# -*- coding: utf-8 -*-

from jsk_robot_startup.msg import Email
import rosgraph
import rosnode
import rospy
from std_msgs.msg import Bool, Float32, Int16
import subprocess
from socket import error as socket_error

try:
    from xmlrpc.client import ServerProxy
except ImportError:
    from xmlrpclib import ServerProxy


class EmailSpotCooler(object):
    """
    This class receives /moisture topic
    and calculates the amount of the water in the spot cooler tank.

    This class daily sends email according to the task status.

    Regularly, it resets serial_node.py
    This is because the connection between serial_node.py
    and M5StickC is terminated by deepsleep in M5StickC.
    """

    def __init__(self):
        self.rosserial_name = rospy.get_param('~rosserial_name')
        self.last_communication = None
        self.last_send_email = None
        # Subscribe moisture
        rospy.Subscriber('moisture', Int16, self.moisture_cb)
        rospy.Subscriber('low_battery', Bool, self.low_battery_cb)
        rospy.Subscriber('battery_level', Float32, self.battery_level_cb)
        self.moisture = None  # 0: most moist, 4095: least moist
        self.moisture_thre = 3000
        self.low_bat = False
        self.bat_level = None
        # Publish email
        self.pub = rospy.Publisher('email', Email, queue_size=1)
        # Check status of M5StickC and EARTH sensor
        # When full water, low battery or next day, send email
        rospy.Timer(rospy.Duration(0.5 * 60 * 60), self.check_status)

    def moisture_cb(self, msg):
        self.moisture = msg.data
        rospy.loginfo('I got moisture data: {}'.format(self.moisture))
        self.last_communication = rospy.Time.now()
        # Reset rosserial because the connection is terminated by M5StickC
        # after M5StickC sends topic
        rospy.sleep(3)  # Wait for other callback functions to exit
        self.reset_rosserial()

    def low_battery_cb(self, msg):
        self.low_bat = msg.data
        rospy.loginfo('I got low_battery data: {}'.format(self.low_bat))

    def battery_level_cb(self, msg):
        self.bat_level = msg.data
        rospy.loginfo('I got battery_level data: {}'.format(self.bat_level))

    # Check amount of the water in the tank
    def water_message(self):
        message = ''
        if self.moisture is None or self.moisture > self.moisture_thre:
            message += 'タンクに水は溜まっていません。\n'
        else:
            message += 'タンクに水が溜まっています。交換してください。\n'
        message += '水分量 {} （基準値3000）\n'.format(self.moisture)
        message += '\n'  # end of this section
        return message

    # Check M5StickC battery
    def battery_message(self):
        message = ''
        if self.low_bat:
            message += 'M5StickCのバッテリ残量はわずかです。充電してください。\n'
        else:
            message += 'M5StickCのバッテリ残量は十分です。\n'
        message += 'バッテリ残量 {}[V]'.format(self.bat_level)
        message += '\n'  # end of this section
        return message

    # Check communication status
    def comm_message(self):
        message = ''
        if self.last_communication is None:
            message += 'まだM5StickCと通信が出来ていません。\n'
        elif (rospy.Time.now() - self.last_communication).secs > 24 * 60 * 60:
            message += '1日以上、M5StickCと通信が出来ていません。情報が古い可能性があります。\n'
        else:
            message += '最後に通信した時刻 {} (UNIX time)\n'.format(
                self.last_communication.secs)
        message += '\n'  # end of this section
        return message

    def send_email(self):
        email_msg = Email()
        now = rospy.Time.now()
        email_msg.header.stamp = now
        email_msg.subject = 'スポットクーラーのタンクの水量'

        body = ''
        body += self.water_message()
        body += self.battery_message()
        body += self.comm_message()
        # Publish Email
        email_msg.body = body
        self.pub.publish(email_msg)
        self.last_send_email = rospy.Time.now()
        rospy.loginfo('Send email')
        # Reset rosserial regularly to avoid failing communication eternally
        self.reset_rosserial()

    def check_status(self, event):
        # Send email as soon as possible when the water is full
        if self.moisture < self.moisture_thre:
            self.send_email()
            rospy.loginfo('Send email because tank water is full')
        # Send email if battery is low
        if self.low_bat:
            self.send_email()
            rospy.loginfo('Send email because battery is low')
        # Send email when this program starts or email is not sent for a day
        if self.last_send_email is None:
            self.send_email()
            rospy.loginfo('Send email at first time')
        elif (rospy.Time.now() - self.last_send_email).secs > 24 * 60 * 60:
            self.send_email()
            rospy.loginfo(
                'Send email because email have not been sent for a day')
        else:
            rospy.loginfo('Timer is called, but do not send email')

    # Reset rosserial by sending SIGTERM to rosserial
    # See https://answers.ros.org/question/271776/how-can-i-retrieve-a-list-of-process-ids-of-ros-nodes/  # NOQA
    # rosnode.kill_nodes() cannot kill serial_node.py absolutely
    def reset_rosserial(self):
        ID = '/rosnode'
        master = rosgraph.Master(ID)
        node_api = rosnode.get_api_uri(
            master, self.rosserial_name, skip_cache=True)
        node = ServerProxy(node_api)
        try:
            pid = rosnode._succeed(node.getPid(ID))
        except socket_error as serr:
            rospy.logerr(serr)
            rospy.logerr('Skip killing rosserial')
        else:
            # To kill rosserial completely, call kill twice
            subprocess.call(['kill', str(pid)])
            rospy.sleep(5)
            subprocess.call(['kill', str(pid)])
            rospy.loginfo('Reset rosserial by sending SIGTERM to rosserial')


if __name__ == '__main__':
    rospy.init_node('email_spot_cooler')
    esc = EmailSpotCooler()
    rospy.spin()
