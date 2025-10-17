#!/usr/bin/env python

import argparse
import math
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String


def test_look_at():
    scale = rospy.get_param('~scale', 10.0)
    msg = Point()
    pub = rospy.Publisher("/eye_display/look_at", Point, queue_size=1)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        msg.x = scale * 0
        msg.y = scale * 0
        rospy.loginfo("Move to center {}".format(msg))
        pub.publish(msg)
        rate.sleep()

        msg.x = scale * 1.0
        msg.y = scale * 0
        rospy.loginfo("Move to right {}".format(msg))
        pub.publish(msg)
        rate.sleep()

        msg.x = scale * 0
        msg.y = scale * 1.0
        rospy.loginfo("Move to bottom {}".format(msg))
        pub.publish(msg)
        rate.sleep()

        msg.x = scale * -1.0
        msg.y = scale * 0
        rospy.loginfo("Move to left {}".format(msg))
        pub.publish(msg)
        rate.sleep()

        msg.x = scale * 0
        msg.y = scale * -1.0
        rospy.loginfo("Move to top {}".format(msg))
        pub.publish(msg)
        rate.sleep()

def test_eye_status_command():
    loop = 0
    msg = String()
    pub = rospy.Publisher("/eye_display/eye_status", String, queue_size=1)

    # set eye_status to normal
    rospy.sleep(0.5)  # wait for subscriber
    pub.publish(String(data="normal"))

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        phase = (loop % 100)/100.0
        if (int(loop / 100) % 3) == 0:  # circle
            msg.data = \
                "eye_asset_default_pos_x: normal: iris: {:.1f}\n".format(30*math.sin(2*math.pi*phase)) + \
                "eye_asset_default_pos_y: normal: iris: {:.1f}\n".format(30*math.cos(2*math.pi*phase)) + \
                "eye_asset_default_theta: normal: iris: {:.1f}\n".format(360*phase)
        elif (int(loop / 100) % 3)  == 1:
            msg.data = "eye_asset_default_zoom: normal: iris: {:.1f}\n".format(math.sin(2*math.pi*phase)+1.0)
        elif (int(loop / 100) % 3)  == 2:
            msg.data = "eye_asset_default_zoom: normal: upperlid: {:.1f}\n".format(30*math.sin(2*math.pi*phase)+7)
        #print ((int(loop / 100) % 3), phase)
        rospy.loginfo("'{}'".format(msg))
        pub.publish(msg)
        rate.sleep()
        loop = loop + 1

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-a", "--advanced", action="store_true", help="test advanced eye status control mode")

    args = parser.parse_args()

    rospy.init_node("pub_eye_status")

    if not args.advanced:
        test_look_at()
    else:
        test_eye_status_command()


if __name__ == "__main__":
    main()
