#!/usr/bin/env python

from gdrive_ros.gdrive_ros_client import GDriveROSClient

try:
    from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat
except ImportError:
    # jsk_common < 3.0 has no jsk_ros1_ros2_compat, so this is ROS1 only:
    # forward ros_compat.xxx(node, ...) to rospy.xxx(...)
    import rospy
    class ros_compat(object):
        get_param = staticmethod(lambda node, *args, **kwargs: rospy.get_param(*args, **kwargs))
        init_node = staticmethod(lambda name, **kwargs: rospy.init_node(name, **kwargs))
        loginfo = staticmethod(lambda node, *args, **kwargs: rospy.loginfo(*args, **kwargs))


def main():

    node = ros_compat.init_node('sample_gdrive_rospy_client')

    file_name = ros_compat.get_param(node, '~file_name')
    file_title = ros_compat.get_param(node, '~file_title')
    parents_path = ros_compat.get_param(node, '~parents_path')

    client = GDriveROSClient(node)

    client.wait_for_gdrive_server()

    ros_compat.loginfo(node, 'Uploading files...')
    ret = client.upload_file(file_name, file_title, parents_path=parents_path)
    ros_compat.loginfo(node, 'Result: {}'.format(ret))

    ros_compat.loginfo(node, 'Uploading files...')
    ret = client.upload_multiple_files([file_name], [file_title], parents_path=parents_path)
    ros_compat.loginfo(node, 'Result: {}'.format(ret))

    ros_compat.loginfo(node, 'Successfully finished.')


if __name__=='__main__':
    main()
