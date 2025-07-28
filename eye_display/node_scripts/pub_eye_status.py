#!/usr/bin/env python

import argparse
import signal
import sys

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



def signal_handler(sig, frame):
    rospy.loginfo("Ctrl+C detected, shutting down...")
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, signal_handler)

    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--dual", action="store_true", help="Enable dual mode")
    parser.add_argument("-n", "--names", nargs="+", help="List of eye status names")
    parser.add_argument("-r", "--rate", type=float, default=1.0/3, help="Set the rate")

    args = parser.parse_args()

    rospy.init_node("pub_eye_status")

    eye_status_names = args.names
    if args.dual:
        pubs = [rospy.Publisher("/right/eye_display/eye_status", String, queue_size=10, latch=True),
               rospy.Publisher("/left/eye_display/eye_status", String, queue_size=10, latch=True)]
        if eye_status_names is None:
            eye_status_names = rospy.get_param("/right/eye_display/eye_asset/names", NAMES)
    else:
        pubs = [rospy.Publisher("/eye_display/eye_status", String, queue_size=10, latch=True)]
        if eye_status_names is None:
            eye_status_names = rospy.get_param("/eye_display/eye_asset/names", NAMES)
    rospy.loginfo("eye status names {}".format(eye_status_names))


    rate = rospy.Rate(args.rate)
    index = 0
    while not rospy.is_shutdown():
        name = eye_status_names[index % len(eye_status_names)]
        rospy.loginfo("Publishing eye_status: {}".format(name))
        [pub.publish(String(name)) for pub in pubs]
        rate.sleep()
        index += 1


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
