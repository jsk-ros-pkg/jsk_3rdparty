#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from spatial_audio_msgs.srv import PlaySpatialAudio, PlaySpatialAudioRequest

def main():
    rospy.init_node( 'simple_client' )
    # parameters
    source_id = rospy.get_param( '~id', 'source_id' )
    source_frame_id  = rospy.get_param( '~frame_id', 'source_sinwave' )
    source_topicname_audio = rospy.get_param( '~source_topicname_audio' )
    source_topicname_info = rospy.get_param( '~source_topicname_info' )
    source_service_name  = '~service'
    # create a request
    req = PlaySpatialAudioRequest()
    req.header.frame_id = source_frame_id
    req.id = source_id
    req.action = req.ADD
    req.pose.orientation.x = 1
    req.stream_topic_audio = source_topicname_audio
    req.stream_topic_info = source_topicname_info
    # service call
    rospy.wait_for_service( source_service_name );
    try:
        service = rospy.ServiceProxy( source_service_name, PlaySpatialAudio )
        res = service( req )
        return res.is_success
    except rospy.ServiceException, e:
        rospy.logerr( 'Service call failed: {}'.format(e) )
        return False

if __name__ == '__main__':
    ret = main()
    rospy.loginfo( 'ret = {}'.format(ret) )
