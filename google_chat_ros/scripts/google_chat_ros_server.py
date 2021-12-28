#!/usr/bin/env python
import actionlib
import gdown
from google_chat_ros.google_chat import GoogleChatRESTClient
from google_chat_ros.google_chat import GoogleChatHTTPSServer
from google_chat_ros.msg import *
from google_chat_ros.msg import SendMessageAction
from google_chat_ros.msg import SendMessageFeedback
from google_chat_ros.msg import SendMessageResult
import requests
from requests.exceptions import Timeout
import rospy


class GoogleChatROS(object):
    """
    Send request to Google Chat REST API via ROS
    """
    def __init__(self):
        recieving_chat_mode = rospy.get_param('~recieving_mode') # select from 'dialogflow', 'url', 'none'

        # For REST, sending message 
        rest_keyfile = rospy.get_param('~google_cloud_credentials_json')
        self._client = GoogleChatRESTClient(rest_keyfile)
        rospy.loginfo("Starting Google Chat REST service...")
        try:
            self._client.build_service() # Start google chat authentication and service
            rospy.loginfo("Succeeded in starting Google Chat REST service")
        except Exception as e:
            rospy.logwarn("Failed to start Google Chat REST service")
            rospy.logerr(e)

        # For POST, recieving message
        if recieving_chat_mode in ("url", "dialogflow"):
            self.host = rospy.get_param('~host')
            self.port = int(rospy.get_param('~port'))
            self.ssl_certfile = rospy.get_param('~ssl_certfile')
            self.ssl_keyfile = rospy.get_param('~ssl_keyfile')
            self.download_timeout = rospy.get_param('~download_data_timeout')
            self.download_data = rospy.get_param('~download_data')
            self.download_avatar = rospy.get_param('~download_avatar')
            self._recieve_message_pub = rospy.Publisher("~recieve", MessageEvent, queue_size=1)
            self._space_activity_pub = rospy.Publisher("~space_activity", SpaceEvent, queue_size=1)
            rospy.loginfo("Starting Google Chat HTTPS server...")
            self._server = GoogleChatHTTPSServer(
                self.host, self.port, self.ssl_certfile, self.ssl_keyfile, callback=self.https_post_cb)
            self._server.run()
        # elif recieving_chat_mode == "dialogflow":
        #     # TODO: subscribe dialogflow msg and pipe to chat message
        #     pass
        elif recieving_chat_mode == "none":
            pass
        else:
            rospy.logerr("Please choose recieving_mode param from dialogflow, https, none.")

        rospy.on_shutdown(self.killnode)

        # ROS ActionLib for REST
        self._as = actionlib.SimpleActionServer(
            '~send', SendMessageAction,
            execute_cb=self.rest_cb, auto_start=False
        )
        self._as.start()

    def killnode(self):
        self._server.kill()

    def rest_cb(self, goal):
        feedback = SendMessageFeedback()
        result = SendMessageResult()
        r = rospy.Rate(1)
        success = True
        # start executing the action
        space = goal.space
        message_type = goal.message_type
        content = goal.content
        try:
            # establish the service
            self._client.build_service()
            if message_type == 'text':
                rospy.loginfo("Send text type message")
                feedback.status = str(
                    self._client.send_text(
                        space=space,
                        text=content
                    ))
            elif message_type == 'card':
                rospy.loginfo("Send card type message")
                feedback.status = str(
                    self._client.send_card(
                        space=space,
                        content=content
                    ))
        except Exception as e:
            rospy.logerr(str(e))
            feedback.status = str(e)
            success = False
        finally:
            self._as.publish_feedback(feedback)
            r.sleep()
            result.done = success
            self._as.set_succeeded(result)

    def https_post_cb(self, event):
        """Parse Google Chat API json content and publish as a ROS Message.
        See https://developers.google.com/chat/api/reference/rest 
        to check what contents are included in the json.
        :param event: A google Chat API POST request json content. 
        See https://developers.google.com/chat/api/guides/message-formats/events#event_fields for details.
        :rtype: None
        """
        # rospy.loginfo(str(event))
        # event/eventTime
        event_time = event['eventTime']
        # event/space
        space = Space()
        space.name = event['space']['name']
        space.room = True if event['space']['type'] == "ROOM" else False
        if space.room:
            space.display_name = event['space']['displayName']
        space.dm = True if event['space']['type'] == "DM" else False
        # event/user
        user = self._get_user_info(event['user'])
        if event['type'] == 'ADDED_TO_SPACE' or event['type'] == 'REMOVED_FROM_SPACE':
            msg = SpaceEvent()
            msg.event_time = event_time
            msg.space = space
            msg.user = user
            msg.added = True if event['type'] == "ADDED_TO_SPACE" else False
            msg.removed = True if event['type'] == "REMOVED_FROM_SPACE" else False
            self._space_activity_pub.publish(msg)
            return
        elif event['type'] == 'MESSAGE':
            msg = MessageEvent()
            msg.event_time = event_time
            msg.space = space
            msg.user = user
            message = Message()
            message_content = event['message']
            message.name = message_content.get('name')
            # event/message/sender
            message.sender = self._get_user_info(message_content['sender'])
            message.create_time = message_content.get('createTime')
            message.text = message_content.get('text')
            message.thread_name = message_content.get('thread').get('name') # TODO maybe error if not exists
            # event/messsage/annotations
            if 'annotations' in message_content:
                for item in message_content['annotations']:
                    annotation = Annotation()
                    annotation.length = item.get('length')
                    annotation.start_index = item.get('startIndex')
                    annotation.mention = True if item.get('type') == 'USER_MENTION' else False
                    if annotation.mention:
                        annotation.user = self._get_user_info(item['userMention']['user'])
                    annotation.slash_command = True if item.get('type') == 'SLASH_COMMAND' else False
                    message.annotations.append(annotation)
            message.argument_text = message_content.get('argumentText', '')
            # event/message/attachment
            if 'attachment' in message_content:
                for item in message_content['attachment']:
                    message.attachments.append(self._get_attachment(item))
            msg.message = message
            rospy.logwarn(msg.event_time)
            try:
                self._recieve_message_pub.publish(msg)
            except Exception as e:
                from IPython.terminal import embed; ipshell=embed.InteractiveShellEmbed(config=embed.load_default_config())(local_ns=locals())
            return
        else:
            rospy.logerr("Got unknown event type.")
            return

    def _get_user_info(self, item):
        user = User()
        user.name = item.get('name', '')
        user.display_name = item.get('displayName', '')
        user.avatar_url = item.get('avatarUrl', '')
        if self.download_avatar:
            user.avatar = self._get_image_from_uri(item['avatar'])
        user.email = item.get('email', '')
        user.bot = True if item.get('type') == "BOT" else False
        user.human = True if item.get('type') == "HUMAN" else False
        return user

    def _get_attachment(self, item, download=False):
        attachment = Attachment()
        attachment.name = item.get('name', '')
        attachment.content_name = item.get('contentName', '')
        attachment.content_type = item.get('contentType', '')
        attachment.thumnail_uri = item.get('thumnailUri', '')
        attachment.download_uri = item.get('downloadUri', '')
        attachment.drive_file = True if item.get('source') == 'DRIVE_FILE' else False
        attachment.uploaded_content = True if item.get('source') == 'UPLOADED_CONTENT' else False
        attachment.attachment_resource_name = item.get('attachmentDataRef', {}).get('resourceName', '')
        attachment.drive_field_id = item.get('driveDataRef', {}).get('driveFileId', '')
        # attachment.localpath = item[''] # TODO enable download option
        return attachment

    def _get_image_from_uri(self, uri):
        img = None
        try:
            img = requests.get(uri, timeout=self.download_timeout).content
        except Timeout:
            rospy.logerr("Exceeded timelimit ({} sec) when downloading {}.".format(self.download_timeout, uri))
        except Exception as e:
            rospy.logwarn("Failed to get image from {}".format(uri))
            rospy.logerr(e)
        return img

    def _download_attachment_from_uri(self, uri):        
        return

if __name__ == '__main__':
    rospy.init_node('google_chat', disable_signals=True)
    node = GoogleChatROS()
    rospy.spin()
