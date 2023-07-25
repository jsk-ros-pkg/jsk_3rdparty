#!/usr/bin/env python

from __future__ import print_function

import datetime
from distutils.version import LooseVersion
import os.path
import pkg_resources
import sys
import time

from httplib2 import ServerNotFoundError
from pydrive.auth import GoogleAuth
from pydrive.drive import GoogleDrive
from pydrive.files import ApiRequestError
import rospy

from gdrive_ros.srv import MultipleUpload
from gdrive_ros.srv import MultipleUploadResponse
from gdrive_ros.srv import Upload
from gdrive_ros.srv import UploadResponse


if sys.version_info.major < 3 and \
        LooseVersion(pkg_resources.get_distribution("rsa").version) \
        >= LooseVersion('4.6.0'):
    print('''rsa < 4.6.0 is required:
    pip install oauth2client==4.2.3 rsa==4.5 pydrive==1.3.1
    For more detailed information,
    please read https://github.com/jsk-ros-pkg/jsk_3rdparty/tree/master/gdrive_ros#trouble-shooting
''', file=sys.stderr)
    sys.exit(1)


class GDriveServerNode(object):
    folder_mime_type = 'application/vnd.google-apps.folder'
    folder_url_format = 'https://drive.google.com/drive/folders/{}'
    file_url_format = 'https://drive.google.com/uc?id={}'

    def __init__(self):
        settings_yaml = rospy.get_param('~settings_yaml', None)
        self.share_type = rospy.get_param('~share_type', 'anyone')
        self.share_value = rospy.get_param('~share_value', 'anyone')
        self.share_role = rospy.get_param('~share_role', 'reader')
        self.share_with_link = rospy.get_param('~share_with_link', True)
        auth_max_trial = rospy.get_param('~auth_max_trial', -1)
        auth_wait_seconds = rospy.get_param('~auth_wait_seconds', 10.0)
        if settings_yaml is not None:
            self.gauth = GoogleAuth(settings_yaml)
            # if client_config_file, save_credentials_file is not found, try to find relative to setting_yaml
            dir_path = os.path.dirname(os.path.realpath(settings_yaml))
            for file in ['client_config_file', 'save_credentials_file']:
                if not os.path.isfile(self.gauth.settings[file]):
                    rospy.logwarn("{}:{} is not found, try to use {}".format(file, self.gauth.settings[file], os.path.join(dir_path, self.gauth.settings[file])))
                    self.gauth.settings[file] = os.path.join(dir_path, self.gauth.settings[file])
        else:
            rospy.logerr('param: ~settings_yaml is not correctly set.')
            sys.exit(1)

        rospy.loginfo('Google drive authentication starts.')
        auth_success = False
        auth_count = 0
        while (not auth_success and
                (auth_max_trial < 0 or auth_count < auth_max_trial)):
            try:
                self.gauth.LocalWebserverAuth()
                auth_success = True
            except ServerNotFoundError as e:
                rospy.logerr('Authentication failed: {}'.format(e))
                auth_count = auth_count + 1
                time.sleep(auth_wait_seconds)
        if not auth_success:
            rospy.logerr(
                'Authentication failed {} times.'.format(auth_max_trial))
            sys.exit(1)
        self.gdrive = GoogleDrive(self.gauth)
        rospy.loginfo('Google drive authentication finished.')
        self.upload_server = rospy.Service('~upload', Upload, self._upload_cb)
        self.upload_multi_server = rospy.Service(
            '~upload_multi', MultipleUpload, self._upload_multi_cb)

    def _upload_cb(self, req):
        timestamp = '{0:%Y%m%d%H%M%S}'.format(datetime.datetime.now())
        parents_path = req.parents_path
        parents_id = req.parents_id

        # response initialization
        res = UploadResponse()
        res.success = False
        res.file_id = ''
        res.file_url = ''

        if parents_id and parents_path:
            rospy.logerr('parents_path and parents_id is both set.')
            rospy.logerr(
                'parents_id: {} is selected to upload.'.format(parents_id))
            parents_path = ''

        if parents_path:
            try:
                parents_id = self._get_parents_id(
                    parents_path, mkdir=True)
            except (ValueError, ApiRequestError, ServerNotFoundError) as e:
                rospy.logerr(e)
                rospy.logerr(
                    'Failed to get parents_id: {}'.format(parents_path))
                return res
        # root
        elif parents_id == '' and parents_path == '':
            parents_id = ''

        if req.use_timestamp_folder:
            try:
                parents_id = self._get_parents_id(
                    [timestamp], parents_id=parents_id, mkdir=True)
            except (ValueError, ApiRequestError, ServerNotFoundError) as e:
                rospy.logerr(e)
                rospy.logerr(
                    'Failed to get parents_id: {} in {}'.format(
                        timestamp, self.folder_url_format.format(parents_id)))
                return res

        success, file_id, file_url = self._upload_step(
            req.file_path, req.file_title, parents_id,
            req.use_timestamp_file_title, timestamp)
        res.success = success
        res.file_id = file_id
        res.file_url = file_url
        res.parents_id = parents_id
        res.parents_url = self.folder_url_format.format(parents_id)
        return res

    def _upload_multi_cb(self, req):
        timestamp = '{0:%Y%m%d%H%M%S}'.format(datetime.datetime.now())
        parents_path = req.parents_path
        parents_id = req.parents_id

        # response initialization
        res = MultipleUploadResponse()
        res.successes = [False] * len(req.file_titles)
        res.file_ids = [''] * len(req.file_titles)
        res.file_urls = [''] * len(req.file_titles)

        if parents_id and parents_path:
            rospy.logerr('parents_path and parents_id is both set.')
            rospy.logerr(
                'parents_id: {} is selected to upload.'.format(parents_id))
            parents_path = ''

        if parents_path:
            try:
                parents_id = self._get_parents_id(
                    parents_path, mkdir=True)
            except (ValueError, ApiRequestError, ServerNotFoundError) as e:
                rospy.logerr(e)
                rospy.logerr(
                    'Failed to get parents_id: {}'.format(parents_path))
                return res
        # root
        elif parents_id == '' and parents_path == '':
            parents_id = ''

        if req.use_timestamp_folder:
            try:
                parents_id = self._get_parents_id(
                    [timestamp], parents_id=parents_id, mkdir=True)
            except (ValueError, ApiRequestError, ServerNotFoundError) as e:
                rospy.logerr(e)
                rospy.logerr(
                    'Failed to get parents_id: {} in {}'.format(
                        timestamp, self.folder_url_format.format(parents_id)))
                return res

        for i, (file_path, file_title) in enumerate(
                zip(req.file_paths, req.file_titles)):
            success, file_id, file_url = self._upload_step(
                file_path, file_title, parents_id,
                req.use_timestamp_file_title, timestamp)
            res.successes[i] = success
            res.file_ids[i] = file_id
            res.file_urls[i] = file_url
        res.parents_id = parents_id
        res.parents_url = self.folder_url_format.format(parents_id)
        return res

    def _upload_step(self, file_path, file_title, parents_id,
                     use_timestamp_file_title=False, timestamp=None):
        file_title = file_title if file_title else file_path.split('/')[-1]
        file_path = os.path.expanduser(file_path)
        if use_timestamp_file_title:
            file_title = '{}_{}'.format(timestamp, file_title)

        success = False
        file_id = ''
        file_url = ''
        folder_url = self.folder_url_format.format(parents_id)
        try:
            file_id = self._upload_file(
                file_path, file_title, parents_id=parents_id)
            file_url = self.file_url_format.format(file_id)
            success = True
            rospy.loginfo(
                'Success to upload: {} -> {}'.format(file_path, file_url))
        except (OSError, ApiRequestError, ServerNotFoundError) as e:
            rospy.logerr(e)
            rospy.logerr(
                'Failed to upload: {} -> {}'.format(file_path, folder_url))
        return success, file_id, file_url

    def _upload_file(self, file_path, file_title, parents_id=None):
        if not os.path.exists(file_path):
            raise OSError('File not found: {}'.format(file_path))
        rospy.loginfo('Start uploading a file: {}'.format(file_title))
        if parents_id:
            gfile = self.gdrive.CreateFile(
                {'parents': [{'id': parents_id}]})
        else:
            gfile = self.gdrive.CreateFile()
        gfile.SetContentFile(file_path)
        gfile['title'] = file_title
        gfile.Upload()
        gfile.InsertPermission({
            'type': self.share_type,
            'value': self.share_value,
            'role': self.share_role,
            'withLink': self.share_with_link,
        })
        rospy.loginfo('Finish uploading a file: {}'.format(file_title))
        return gfile['id']

    def _upload_folder(self, folder_title, parents_id=None):
        rospy.loginfo('Start making a folder: {}'.format(folder_title))
        if parents_id:
            gfolder = self.gdrive.CreateFile(
                {'title': folder_title,
                 'parents': [{'id': parents_id}],
                 'mimeType': 'application/vnd.google-apps.folder'})
        else:
            gfolder = self.gdrive.CreateFile(
                {'title': folder_title,
                 'mimeType': 'application/vnd.google-apps.folder'})
        gfolder.Upload()
        rospy.loginfo('Finish making a folder: {}'.format(folder_title))
        return gfolder['id']

    def _get_parents_id(self, parents_path, parents_id=None, mkdir=False):
        if parents_path == '':
            return None

        if not isinstance(parents_path, list):
            parents_path = [p for p in parents_path.split('/') if p != '']

        folder_title = parents_path[0]
        parent = parents_id if parents_id else 'root'
        gfolders = self.gdrive.ListFile(
            {'q': "'{}' in parents and mimeType = '{}' and title = '{}' and trashed=false".format(parent, self.folder_mime_type, folder_title)}).GetList()
        if len(parents_path) == 1:
            if len(gfolders) > 0:
                return gfolders[0]['id']
            if mkdir:
                folder_id = self._upload_folder(
                    folder_title, parents_id=parents_id)
                return folder_id
            else:
                raise ValueError(
                    'Folder is not found: {}'.format(folder_title))
        else:
            if len(gfolders) > 0 or mkdir:
                if len(gfolders) > 0:
                    next_parents_id = gfolders[0]['id']
                elif mkdir:
                    next_parents_id = self._upload_folder(
                        folder_title, parents_id=parents_id)
                folder_id = self._get_parents_id(
                    parents_path[1:], parents_id=next_parents_id,
                    mkdir=mkdir)
                return folder_id
            else:
                raise ValueError('folder is not found: {}', folder_title)


if __name__ == '__main__':
    rospy.init_node('gdrive_server')
    node = GDriveServerNode()
    rospy.spin()
