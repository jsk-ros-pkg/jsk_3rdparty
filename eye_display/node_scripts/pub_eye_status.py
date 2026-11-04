#!/usr/bin/env python

import rospy
from std_msgs.msg import String

NAMES = ["normal",
         "blink",
         "surprised",
         "sleepy",
         "angry",
         "sad",
         "happy"
         # "troubled",
         # "delighted",
         # "expecting",
         # "heart",
         # "shine",
         # "flustrated",
         # "boring",
    ]


def main():
    rospy.init_node("pub_eye_status")

    pub = rospy.Publisher("/eye_display/eye_status", String, queue_size=10)
    eye_status_names = rospy.get_param("/eye_display/eye_asset/names", NAMES)
    rospy.loginfo("eye status names {}".format(eye_status_names))

    rate = rospy.Rate(1.0/3)
    index = 0
    while not rospy.is_shutdown():
        name = eye_status_names[index % len(eye_status_names)]
        rospy.loginfo("eye_status: {}".format(name))
        pub.publish(String(name))
        rate.sleep()
        index += 1


if __name__ == "__main__":
    main()
