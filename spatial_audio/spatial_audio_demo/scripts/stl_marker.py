#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from visualization_msgs.msg import Marker

def main():

    rospy.init_node( 'simple_stl_marker' )
    # parameteres
    marker_frame_id = rospy.get_param( '~frame_id', 'map' )
    marker_ns = rospy.get_param( '~ns', '' )
    marker_id = rospy.get_param( '~id', 0 )
    marker_scale = rospy.get_param( '~scale', 1.0 )
    marker_mesh_resource = rospy.get_param( '~mesh_resource' )
    marker_publish_rate = rospy.get_param( '~publish_rate',  10 )
    marker_color_a = rospy.get_param( '~color_a', 1.0 )
    marker_color_r = rospy.get_param( '~color_r', 0.0 )
    marker_color_g = rospy.get_param( '~color_g', 1.0 )
    marker_color_b = rospy.get_param( '~color_b', 1.0 )
    # publisher
    pub = rospy.Publisher('~output', Marker, queue_size = 10)
    # create marker object
    marker = Marker()
    marker.header.frame_id = marker_frame_id
    marker.ns = marker_ns
    marker.id = marker_id
    marker.type = Marker.MESH_RESOURCE
    marker.action = Marker.ADD
    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = marker_scale
    marker.scale.y = marker_scale
    marker.scale.z = marker_scale
    marker.color.a = marker_color_a
    marker.color.r = marker_color_r
    marker.color.g = marker_color_g
    marker.color.b = marker_color_b
    marker.mesh_resource = marker_mesh_resource
    # spin
    r = rospy.Rate( marker_publish_rate )
    while not rospy.is_shutdown():
        rospy.logdebug('publish a marker')
        marker.header.stamp = rospy.Time.now()
        pub.publish( marker )
        r.sleep()

if __name__ == '__main__':
    main()
