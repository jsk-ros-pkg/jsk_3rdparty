#!/usr/bin/env python
# -*- coding: utf-8 -*-

from jsk_robot_startup.msg import EmailBody
from m5stack_ros import EmailRosserial
import rospy
from std_msgs.msg import Int16


class EmailSpotCooler(EmailRosserial):
    """
    This class receives /moisture topic
    and calculates the amount of the water in the spot cooler tank.
    The result is sent via email.
    """

    def __init__(self):
        super(EmailSpotCooler, self).__init__()
        self.device_name = 'M5StickC'
        self.subject = 'スポットクーラーのタンクの水量'
        # Subscribe moisture
        rospy.Subscriber('moisture', Int16, self.moisture_cb)
        self.moisture = None  # 0: most moist, 4095: least moist
        self.moisture_thre = 3000

    def moisture_cb(self, msg):
        self.moisture = msg.data
        rospy.loginfo('I got moisture data: {}'.format(self.moisture))
        self.last_communication = rospy.Time.now()

    # When full water, low battery or next day, send email
    def check_status(self, event):
        # Send email as soon as possible when the water is full
        if self.moisture is not None and self.moisture < self.moisture_thre:
            self.send_email()
            rospy.loginfo('Send email because tank water is full')
        # send email when low battery or next day
        super(EmailSpotCooler, self).check_status(event)

    # Check amount of the water in the tank
    def water_email_body(self):
        message = ''
        if self.moisture is None or self.moisture > self.moisture_thre:
            message += 'タンクに水は溜まっていません。\n'
        else:
            message += 'タンクに水が溜まっています。交換してください。\n'
        message += '水分量 {} （基準値3000）\n'.format(self.moisture)
        message += '\n'  # end of this section
        email_body = EmailBody()
        email_body.type = 'text'
        email_body.message = message
        return email_body

    def create_email_body(self):
        body = super(EmailSpotCooler, self).create_email_body()
        body.append(self.water_email_body())
        return body


if __name__ == '__main__':
    rospy.init_node('email_spot_cooler')
    esc = EmailSpotCooler()
    rospy.spin()
