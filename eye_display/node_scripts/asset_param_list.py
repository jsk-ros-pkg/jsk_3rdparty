#!/usr/bin/env python

import os
import rospy
import rosparam
from rosserial_msgs.srv import RequestParam, RequestParamResponse

# This node provides a list of parameters related to eye_asset.
# When getParam is called from the ESP32 with non-existent names, it takes a long time to timeout and returns an error.
# To prevent this delay, we first retrieve the list of available parameter names before requesting their values.

def main():
    rospy.init_node('asset_param_list')

    # get eye_asset param
    param_ns = rospy.get_namespace().rstrip('/')
    eye_asset_names = rospy.get_param("{}/eye_asset/names".format(param_ns))

    for name in eye_asset_names:
        eye_asset_param_names = rospy.get_param("{}/eye_asset/{}".format(param_ns, name)).keys()

        eye_types = []
        for key in eye_asset_param_names:
            # image path : cehck if key start with path_
            if key.startswith("path_"):
                eye_type = key[len("path_"):]
                eye_types.append(eye_type)
        rospy.set_param("{}/eye_asset_{}_eye_types".format(param_ns, name), eye_types)

        positions = []
        for eye_type in eye_types:
            positions.extend([n for n in eye_asset_param_names if n.startswith("{}_position".format(eye_type))])
        if not positions:
            positions.append('NONE')
        rospy.set_param("{}/eye_asset_{}_positions".format(param_ns, name), positions)

        rotations = []
        for eye_type in eye_types:
            rotations.extend([n for n in eye_asset_param_names if n.startswith("{}_rotation".format(eye_type))])
        if not rotations:
            rotations.append('NONE')
        rospy.set_param("{}/eye_asset_{}_rotations".format(param_ns, name), rotations)

        defaults = []
        for eye_type in eye_types:
            defaults.extend([n for n in eye_asset_param_names if n.startswith("{}_default".format(eye_type))])
        if not defaults:
            defaults.append('NONE')
        rospy.set_param("{}/eye_asset_{}_defaults".format(param_ns, name), defaults)

        zooms = []
        for eye_type in eye_types:
            zooms.extend([n for n in eye_asset_param_names if n.startswith("{}_zoom".format(eye_type))])
        if not zooms:
            zooms.append('NONE')
        rospy.set_param("{}/eye_asset_{}_zooms".format(param_ns, name), zooms)

if __name__ == "__main__":
    main()
