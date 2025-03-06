#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
from cv_bridge import CvBridge
from jsk_robot_startup.msg import EmailBody
from m5stack_ros import EmailRosserial
import os
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import UInt16


class EmailFridgeContents(EmailRosserial):
    """
    This class receives /timer_cam_image topic and send email with the image
    The purpose is to keep the M5 TimerCam in the refrigerator
    and monitor the contents regularly.
    """

    def __init__(self):
        super(EmailFridgeContents, self).__init__()
        self.device_name = 'M5TimerCam'
        self.subject = '冷蔵庫の中の状態'
        # Subscribers
        rospy.Subscriber('timer_cam_image', Image, self.image_cb)
        self.img_file_path = '/tmp/email_fridge_contents.png'
        self.gas_info = [
            {'Name': 'NO2', 'Topic': 'gas_v2_102b', 'Concentration': None},
            {'Name': 'C2H5CH', 'Topic': 'gas_v2_302b', 'Concentration': None},
            {'Name': 'VOC', 'Topic': 'gas_v2_502b', 'Concentration': None},
            {'Name': 'CO', 'Topic': 'gas_v2_702b', 'Concentration': None}]
        gas_topic_names = [gas['Topic'] for gas in self.gas_info]
        for gas_topic_name in gas_topic_names:
            rospy.Subscriber(
                gas_topic_name, UInt16, self.gas_cb, gas_topic_name)
        rospy.Subscriber('tof', UInt16, self.tof_cb)
        self.tof = None
        self.tof_threshold = 30
        # Do not send old image
        if os.path.exists(self.img_file_path):
            os.remove(self.img_file_path)

    def image_cb(self, msg):
        self.image = msg
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(msg)
        cv2.imwrite(self.img_file_path, img)
        self.update_last_communication()

    def gas_cb(self, msg, gas_topic_name):
        for gas_info in self.gas_info:
            if gas_info['Topic'] == gas_topic_name:
                gas_info['Concentration'] = msg.data
        self.update_last_communication()

    def tof_cb(self, msg):
        self.tof = msg.data
        self.update_last_communication()

    # When door is left open, low battery or next day, send email
    def check_status(self, event):
        if self.tof is not None and self.tof > self.tof_threshold:
            self.send_email()
            rospy.loginfo('Send email because the fridge door is left open')
        super(EmailFridgeContents, self).check_status(event)

    # Check the contents in the fridge by camera
    def email_image_body(self):
        email_body = EmailBody()
        if os.path.exists(self.img_file_path):
            email_body.type = 'img'
            email_body.message = '冷蔵庫の中身の写真です\n'
            email_body.file_path = self.img_file_path
            email_body.img_size = 100
        else:
            email_body.type = 'text'
            email_body.message = '冷蔵庫の中身の写真は届いていません\n'
        email_body.message += '\n'  # end of this section
        return email_body

    # Check the gas level in the fridge
    def email_gas_body(self, index):
        email_body = EmailBody()
        email_body.type = 'text'
        gas_info = self.gas_info[index]
        if gas_info['Concentration'] is None:
            email_body.message = '冷蔵庫の{}のデータは届いていません\n'.format(
                gas_info['Name'])
        else:
            email_body.message = '冷蔵庫の{}の濃度は{}です\n'.format(
                gas_info['Name'], gas_info['Concentration'])
        email_body.message += '\n'  # end of this section
        return email_body

    # Check the fridge door state by tof
    def email_tof_body(self):
        email_body = EmailBody()
        email_body.type = 'text'
        if self.tof is None:
            email_body.message = '冷蔵庫のToFのデータは届いていません\n'
        else:
            if self.tof > self.tof_threshold:
                email_body.message += '冷蔵庫の扉が開いたままです。(ToF: {})\n'.format(
                    self.tof)
            else:
                email_body.message += '冷蔵庫の扉は閉じられています。(ToF: {})\n'.format(
                    self.tof)
        email_body.message += '\n'  # end of this section
        return email_body

    def create_email_body(self):
        body = super(EmailFridgeContents, self).create_email_body()
        for i in range(len(self.gas_info)):
            body.append(self.email_gas_body(i))
        body.append(self.email_tof_body())
        body.append(self.email_image_body())
        return body

    def send_email(self):
        super(EmailFridgeContents, self).send_email()
        # Do not send the same image twice
        if os.path.exists(self.img_file_path):
            os.remove(self.img_file_path)


if __name__ == '__main__':
    rospy.init_node('email_spot_cooler')
    efc = EmailFridgeContents()
    rospy.spin()
