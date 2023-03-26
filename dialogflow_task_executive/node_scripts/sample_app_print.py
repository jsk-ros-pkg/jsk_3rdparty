#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import rospy

def main():

    rospy.init_node('sample_app_print')

    place = rospy.get_param('~place')

    timeout = rospy.Time.now() + rospy.Duration(10)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and rospy.Time.now() < timeout:
        rate.sleep()
        rospy.loginfo('I come from {}'.format(place))


if __name__=='__main__':
    main()
