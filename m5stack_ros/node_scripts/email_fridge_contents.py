#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
from cv_bridge import CvBridge
from jsk_robot_startup.msg import EmailBody
from m5stack_ros import EmailRosserial
import os
import rospy
from sensor_msgs.msg import Image


class EmailFridgeContents(EmailRosserial):
    """
    This class receives /timer_cam_image topic and send email with the image
    The purpose is to keep the M5 TimerCam in the refrigerator
    and monitor the contents regularly.
    """

    def __init__(self):
        super(EmailFridgeContents, self).__init__()
        self.device_name = 'M5TimerCam'
        self.subject = '冷蔵庫の中の写真'
        # Subscribe image
        rospy.Subscriber('timer_cam_image', Image, self.image_cb)
        self.img_file_path = '/tmp/email_fridge_contents.png'
        if os.path.exists(self.img_file_path):
            os.remove(self.img_file_path)

    def image_cb(self, msg):
        self.image = msg
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(msg)
        cv2.imwrite(self.img_file_path, img)

    # When low battery or next day, send email
    def check_status(self, event):
        super(EmailFridgeContents, self).check_status(event)

    # Check the contents in the fridge
    def fridge_email_body(self):
        email_body = EmailBody()
        email_body.type = 'img'
        email_body.message = '冷蔵庫の中身の写真です'
        email_body.file_path = self.img_file_path
        email_body.img_size = 100
        return email_body

    def create_email_body(self):
        body = super(EmailFridgeContents, self).create_email_body()
        body.append(self.fridge_email_body())
        return body

    def send_email(self):
        super(EmailFridgeContents, self).send_email()
        if os.path.exists(self.img_file_path):
            os.remove(self.img_file_path)


if __name__ == '__main__':
    rospy.init_node('email_spot_cooler')
    efc = EmailFridgeContents()
    rospy.spin()
