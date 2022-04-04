import rospy
from gdrive_ros.srv import MultipleUpload
from gdrive_ros.srv import MultipleUploadRequest
from gdrive_ros.srv import Upload
from gdrive_ros.srv import UploadRequest


class GDriveROSClient:

    def __init__(self):

        self.srv_upload = rospy.ServiceProxy(
                '/gdrive_server/upload',
                Upload
                )
        self.srv_upload_multi = rospy.ServiceProxy(
                '/gdrive_server/upload_multi',
                MultipleUpload
                )

    def wait_for_gdrive_server(self,timeout=None):
        if timeout is None:
            rospy.wait_for_service('/gdrive_server/upload')
            rospy.wait_for_service('/gdrive_server/upload_multi')

        try:
            rospy.wait_for_service('/gdrive_server/upload', timeout=timeout)
            rospy.wait_for_service('/gdrive_server/upload_multi', timeout=timeout)
            return True
        except rospy.ROSException as e:
            rospy.logerr('Error: {}'.format(e))
            return False

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
