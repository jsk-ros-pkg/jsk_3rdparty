#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point


def main():
    rospy.init_node("pub_eye_status")
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


if __name__ == "__main__":
    main()
