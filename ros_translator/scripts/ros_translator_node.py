#!/usr/bin/env python

import rospy
from ros_translator.core import ROSTranslator

if __name__ == '__main__':

  rospy.init_node('ros_translator')
  node = ROSTranslator()
  rospy.spin()
