#!/usr/bin/env python

import rospy
from std_msgs.msg import String


class DEMO(object):

  def __init__(self):

    self.sub = rospy.Subscriber('~text', String, self.callback)

  def callback(self, msg):

    rospy.loginfo('Translated text: {}'.format(msg.data))


if __name__ == '__main__':
  rospy.init_node('demo_output')
  node = DEMO()
  rospy.spin()
