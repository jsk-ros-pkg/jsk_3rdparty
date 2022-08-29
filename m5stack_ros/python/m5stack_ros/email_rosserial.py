# -*- coding: utf-8 -*-
from datetime import datetime
from jsk_robot_startup.msg import Email, EmailBody
import rosgraph
import rosnode
import rospy
from std_msgs.msg import Float32
import subprocess
from socket import error as socket_error

try:
    from xmlrpc.client import ServerProxy
except ImportError:
    from xmlrpclib import ServerProxy


class EmailRosserial(object):
    """
    This is base class to daily sends email according to the M5 device status.

    Regularly, it resets serial_node.py
    This is because the connection between serial_node.py and the M5 device
    is terminated by the M5 device deepsleep.
    """
    def __init__(self):
        self.rosserial_name = rospy.get_param('~rosserial_name')
        self.last_communication = None
        self.last_send_email = None
        # Battery info
        rospy.Subscriber('battery_level', Float32, self.battery_level_cb)
        self.low_bat = False
        self.bat_level = None
        # Publish email
        self.email_duration = rospy.get_param('~email_duration', 24 * 60 * 60)
        self.pub = rospy.Publisher('email', Email, queue_size=1)
        # Check status of M5 device and sensor and send email if needed
        self.check_duration = rospy.get_param('~check_duration', 30)
        rospy.Timer(rospy.Duration(self.check_duration), self.check_status)
        # Override these variables in child class
        self.device_name = 'M5Stack'
        self.subject = 'Subject'
        self.low_bat_thre = 3.6
        self.sender_address = ''
        self.receiver_address = ''

    def battery_level_cb(self, msg):
        self.bat_level = msg.data
        rospy.loginfo('I got battery_level data: {}'.format(self.bat_level))
        if self.bat_level < self.low_bat_thre:
            self.low_bat = True
        else:
            self.low_bat = False
        self.last_communication = rospy.Time.now()
        # Reset rosserial because the connection is terminated by M5StickC
        # after M5StickC sends topic
        rospy.sleep(3)  # Wait for other callback functions to exit
        self.reset_rosserial()

    # Check M5StickC battery
    def battery_message(self):
        message = ''
        if self.low_bat:
            message += '{}のバッテリ残量はわずかです。充電してください。\n'.format(
                self.device_name)
        else:
            message += '{}のバッテリ残量は十分です。\n'.format(
                self.device_name)
        message += 'バッテリ残量 {}[V]'.format(self.bat_level)
        message += '\n'  # end of this section
        return message

    # Check communication status
    def comm_message(self):
        message = ''
        if self.last_communication is None:
            message += 'まだ{}と通信が出来ていません。\n'.format(self.device_name)
        elif (rospy.Time.now() - self.last_communication).secs > 24 * 60 * 60:
            message += '1日以上、{}と通信が出来ていません。情報が古い可能性があります。\n'.format(
                self.device_name)
        else:
            unix_time_for_jst = self.last_communication.secs + (9 * 60 * 60)
            dt = datetime.utcfromtimestamp(unix_time_for_jst)
            message += '最後に通信した時刻 {} (JST)\n'.format(dt)
        message += '\n'  # end of this section
        return message

    def send_email(self):
        email_msg = Email()
        now = rospy.Time.now()
        email_msg.header.stamp = now
        email_msg.subject = self.subject
        email_msg.sender_address = self.sender_address
        email_msg.receiver_address = self.receiver_address
        email_body = self.create_email_body()
        email_msg.body = email_body
        # Publish Email
        self.pub.publish(email_msg)
        self.last_send_email = rospy.Time.now()
        rospy.loginfo('Send email')
        # Reset rosserial regularly to avoid failing communication eternally
        self.reset_rosserial()

    def create_email_body(self):
        """
        return string of what you want to write in the email body.
        """
        email_body = EmailBody()
        email_body.type = 'text'
        message = ''
        message += self.battery_message()
        message += self.comm_message()
        email_body.message = message
        return [email_body]

    # Low battery or next day, send email
    def check_status(self, event):
        """
        Check M5 device status and send email if needed.
        """
        rospy.loginfo('Check status of M5 device')
        # Send email if battery is low
        if self.low_bat:
            self.send_email()
            rospy.loginfo('Send email because battery is low')
        # Send email when this program starts or email is not sent for a day
        if self.last_send_email is None:
            self.send_email()
            rospy.loginfo('Send email at first time')
            return
        secs_from_last_email = (rospy.Time.now() - self.last_send_email).secs
        if secs_from_last_email > self.email_duration:
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
