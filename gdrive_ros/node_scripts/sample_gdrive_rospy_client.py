#!/usr/bin/env python

import rospy
from gdrive_ros.gdrive_ros_client import GDriveROSClient


def main():

    rospy.init_node('sample_gdrive_rospy_client')

    file_name = rospy.get_param('~file_name')
    file_title = rospy.get_param('~file_title')
    parents_path = rospy.get_param('~parents_path')

    client = GDriveROSClient()

    client.wait_for_gdrive_server()

    rospy.loginfo('Uploading files...')
    ret = client.upload_file(file_name, file_title, parents_path=parents_path)
    rospy.loginfo('Result: {}'.format(ret))

    rospy.loginfo('Uploading files...')
    ret = client.upload_multiple_files([file_name], [file_title], parents_path=parents_path)
    rospy.loginfo('Result: {}'.format(ret))

    rospy.loginfo('Successfully finished.')


if __name__=='__main__':
    main()
