from gdrive_ros.srv import MultipleUpload
from gdrive_ros.srv import Upload

try:
    from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat
    from jsk_ros1_ros2_compat.rospy_rclpy_compat import ROS_VERSION
except ImportError:
    # jsk_common < 3.0 has no jsk_ros1_ros2_compat, so this is ROS1 only:
    # forward ros_compat.xxx(node, ...) to rospy.xxx(...)
    import rospy
    ROS_VERSION = 1
    class ros_compat(object):
        call_service = staticmethod(lambda node, name, srv_type, request, timeout=None: rospy.ServiceProxy(name, srv_type)(request))
        logerr = staticmethod(lambda node, *args, **kwargs: rospy.logerr(*args, **kwargs))

        @staticmethod
        def wait_for_service(node, name, srv_type, timeout=None):
            try:
                rospy.wait_for_service(name, timeout)
                return True
            except rospy.ROSException:
                return False

if ROS_VERSION == 2:
    MultipleUploadRequest = MultipleUpload.Request
    UploadRequest = Upload.Request
else:
    import rospy
    from gdrive_ros.srv import MultipleUploadRequest
    from gdrive_ros.srv import UploadRequest


class GDriveROSClient:

    def __init__(self, node=None):

        self.node = node
        self.srv_upload = lambda req: ros_compat.call_service(
            node, '/gdrive_server/upload', Upload, req)
        self.srv_upload_multi = lambda req: ros_compat.call_service(
            node, '/gdrive_server/upload_multi', MultipleUpload, req)

    def wait_for_gdrive_server(self,timeout=None):
        for name, srv_type in [('/gdrive_server/upload', Upload),
                               ('/gdrive_server/upload_multi', MultipleUpload)]:
            if not ros_compat.wait_for_service(self.node, name, srv_type, timeout):
                ros_compat.logerr(self.node, 'Error: {} is not available'.format(name))
                return False
        return True

    def upload_file(self,
               file_path,
               file_title,
               parents_path='',
               parents_id='',
               use_timestamp_folder=False,
               use_timestamp_file_title=False
               ):
        #
        req = UploadRequest()
        req.file_path = file_path
        req.file_title = file_title
        req.parents_path = parents_path
        req.parents_id = parents_id
        req.use_timestamp_folder = use_timestamp_folder
        req.use_timestamp_file_title = use_timestamp_file_title
        #
        res = self.srv_upload(req)
        #
        return (res.success,
                res.file_id,
                res.file_url,
                res.parents_id,
                res.parents_url)

    def upload_multiple_files(self,
                              file_paths,
                              file_titles,
                              parents_path='',
                              parents_id='',
                              use_timestamp_folder=False,
                              use_timestamp_file_title=False
                              ):
        #
        req = MultipleUploadRequest()
        req.file_paths = file_paths
        req.file_titles = file_titles
        req.parents_path = parents_path
        req.parents_id = parents_id
        req.use_timestamp_folder = use_timestamp_folder
        req.use_timestamp_file_title = use_timestamp_file_title
        #
        res = self.srv_upload_multi(req)
        #
        return (res.successes,
                res.file_ids,
                res.file_urls,
                res.parents_id,
                res.parents_url)
