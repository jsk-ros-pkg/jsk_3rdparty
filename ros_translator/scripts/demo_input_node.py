#!/usr/bin/env python

import rospy
from std_msgs.msg import String


class DEMO(object):

  def __init__(self):

    self.pub = rospy.Publisher('~text', String, queue_size=1)

  def prompt(self):

    while not rospy.is_shutdown():

      text = input('Input any text >')
      msg = String(data=text)
      self.pub.publish(msg)


if __name__ == '__main__':
  rospy.init_node('demo_input')
  node = DEMO()
  node.prompt()
