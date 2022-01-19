#!/usr/bin/env python
import gdown
import json
import os
import requests
from requests.exceptions import Timeout
from requests.exceptions import ConnectionError

# ROS libraries
import actionlib
import ast
from dialogflow_task_executive.msg import DialogResponse
from dialogflow_webhook_ros.msg import OriginalDetectIntentRequest
from gdrive_ros.srv import *
from google_chat_ros.google_chat import GoogleChatRESTClient
from google_chat_ros.google_chat import GoogleChatHTTPSServer
from google_chat_ros.msg import *
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
            # ROS ActionLib
            self._as = actionlib.SimpleActionServer(
                '~send', SendMessageAction,
                execute_cb=self.rest_cb, auto_start=False
        )
            self._as.start()
        except Exception as e:
            rospy.logwarn("Failed to start Google Chat REST service")
            rospy.logerr(e)

        # For POST, recieving message
        if recieving_chat_mode in ("url", "dialogflow"):
            # rosparams
            self.download_data = rospy.get_param('~download_data')
            self.download_directory = rospy.get_param('~download_directory')
            self.download_avatar = rospy.get_param('~download_avatar')
            # ROS publisher
            self._message_activity_pub = rospy.Publisher("~message_activity", MessageEvent, queue_size=1)
            self._space_activity_pub = rospy.Publisher("~space_activity", SpaceEvent, queue_size=1)
            self._card_activity_pub = rospy.Publisher("~card_activity", CardEvent, queue_size=1)

            rospy.loginfo("Starting Google Chat HTTPS server...")
            try:
                if recieving_chat_mode == "url":
                    rospy.loginfo("Expected to get Google Chat Bot URL request")
                    self.host = rospy.get_param('~host')
                    self.port = int(rospy.get_param('~port'))
                    self.ssl_certfile = rospy.get_param('~ssl_certfile')
                    self.ssl_keyfile = rospy.get_param('~ssl_keyfile')
                    self._server = GoogleChatHTTPSServer(
                        self.host, self.port, self.ssl_certfile, self.ssl_keyfile, callback=self.event_cb)
                    rospy.on_shutdown(self.killhttpd) # shutdown https server
                    self._server.run()
                elif recieving_chat_mode == "dialogflow":
                    rospy.loginfo("Expected to get OriginalDetectIntentRequest.msg from dialogflow webhook ros node.")
                    self._sub = rospy.Subscriber("dialogflow_original_application_request", OriginalDetectIntentRequest, self.dialogflow_cb)

            except ConnectionError as e:
                rospy.logwarn("The error occurred while starting HTTPS server")
                rospy.logerr(e)

        elif recieving_chat_mode == "none":
            rospy.logwarn("You cannot recieve Google Chat event because HTTPS server is not running.")

        else:
            rospy.logerr("Please choose recieving_mode param from dialogflow, https, none.")

    def killhttpd(self):
        self._server.kill()

    def rest_cb(self, goal):
        """Get ROS SendMessageAction Goal and send request to Google Chat API.
        :param goal: ROS SendMessageAction Goal
        :rtype: none
        """
        feedback = SendMessageFeedback()
        result = SendMessageResult()
        r = rospy.Rate(1)
        success = True

        json_body = {}
        json_body['text'] = goal.text
        json_body['thread'] = {'name': goal.thread_name}
        json_body['cards'] = []

        # Card
        if goal.cards:
            for card in goal.cards:
                card_body = {}
                # card/header
                header = {}
                header['title'] = card.header.title
                header['subtitle'] = card.header.subtitle
                header['imageStyle'] = 'AVATAR' if card.header.image_style_circular else 'IMAGE'
                if card.header.image_url:
                    header['imageUrl'] = card.header.image_url
                elif card.header.image_filepath:
                    header['imageUrl'] = self._upload_file(card.header.image_filepath)
                # card/sections
                sections = []
                sections = self._make_sections_json(card.sections)
                # card/actions
                card_actions = []
                for card_action_msg in card.card_actions:
                    card_action = {}
                    card_action['actionLabel'] = card_action_msg.action_label
                    card_action['onClick'] = self._make_on_click_json(card_action_msg.on_click)
                    card_actions.append(card_action)
                card_body['header'] = header
                card_body['sections'] = sections
                card_body['cardActions'] = card_actions
                card_body['name'] = card.name
                json_body['cards'].append(card_body)

        try:
            # establish the service
            self._client.build_service()
            rospy.loginfo("Send text type message")
            feedback.status = str(
                self._client.message_request(
                    space=goal.space,
                    json_body=json_body
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

    def event_cb(self, event):
        """Parse Google Chat API json content and publish as a ROS Message.
        See https://developers.google.com/chat/api/reference/rest 
        to check what contents are included in the json.
        :param event: A google Chat API POST request json content. 
        See https://developers.google.com/chat/api/guides/message-formats/events#event_fields for details.
        :rtype: None
        """
        rospy.logdebug("GOOGLE CHAT ORIGINAL JSON EVENT")
        rospy.logdebug(json.dumps(event, indent=2))
        # GET EVENT TYPE
        # event/eventTime
        event_time = event.get('eventTime')
        # event/space
        space = Space()
        space.name = event.get('space').get('name')
        space.room = True if event.get('space').get('type') == "ROOM" else False
        if space.room:
            space.display_name = event.get('space').get('displayName')
        space.dm = True if event.get('space').get('type') == "DM" else False
        # event/user
        user = self._get_user_info(event.get('user'))

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
            msg.message = self._make_message_msg(event)
            self._message_activity_pub.publish(msg)
            return

        elif event['type'] == 'CARD_CLICKED':
            msg = CardEvent()
            msg.event_time = event_time
            msg.space = space
            msg.user = user
            msg.message = self._make_message_msg(event)
            if event.get('action'):
                action = event.get('action')
                msg.action.action_method_name = action.get('actionMethodName')
                if action.get('actionMethodName', {}).get('parameters'):
                    parameters = []
                    for param in action.get('actionMethodName').get('parameters'):
                        action_parameter = ActionParameter()
                        action_parameter.key = param.get('key')
                        action_parameter.value = param.get('value')
                        parameters.append(action_parameter)
                msg.action.parameters = parameters
            self._card_activity_pub.publish(msg)
            return

        else:
            rospy.logerr("Got unknown event type.")
            return

    def dialogflow_cb(self, msg):
        if msg.source == "hangouts":
            ast_data = ast.literal_eval(msg.payload)
            json_dumped = json.dumps(ast_data)
            json_data = json.loads(json_dumped)
            self.event_cb(json_data.get('data', {}).get('event', {}))

    def _make_message_msg(self, event):
        message = Message()
        message_content = event.get('message', '')
        message.name = message_content.get('name', '')

        # event/message/sender
        message.sender = self._get_user_info(message_content.get('sender'))
        message.create_time = message_content.get('createTime', '')
        message.text = message_content.get('text', '')
        message.thread_name = message_content.get('thread', {}).get('name', '')

        # event/messsage/annotations
        if 'annotations' in message_content:
            for item in message_content['annotations']:
                annotation = Annotation()
                annotation.length = int(item.get('length', 0))
                annotation.start_index = int(item.get('startIndex', 0))
                annotation.mention = True if item.get('type') == 'USER_MENTION' else False
                if annotation.mention:
                    annotation.user = self._get_user_info(item.get('userMention').get('user'))
                annotation.slash_command = True if item.get('type') == 'SLASH_COMMAND' else False
                message.annotations.append(annotation)
        message.argument_text = message_content.get('argumentText', '')

        # event/message/attachment
        if 'attachment' in message_content:
            for item in message_content['attachment']:
                message.attachments.append(self._get_attachment(item))

        return message

    def _make_sections_json(self, sections_msg):
        """
        :type msg: list of google_chat_ros.msgs/Section
        :rtype json_body: list of json
        """
        json_body = []
        for msg in sections_msg:
            section = {}
            section['header'] = msg.header
            section['widgets'] = self._make_widget_markups_json(msg.widgets)
            json_body.append(section)
        return json_body

    def _make_widget_markups_json(self, widgets_msg):
        """Make widget markup json lists.
        See https://developers.google.com/chat/api/reference/rest/v1/cards#widgetmarkup for details.
        :rtype widgets_msg: list of google_chat_ros.msgs/WidgetMarkup
        :rtype json_body: list of json
        """
        json_body = []
        for msg in widgets_msg:
            is_text = bool(msg.text_paragraph)
            is_image = bool(msg.image.image_uri) or bool(msg.image.localpath)
            is_keyval = bool(msg.key_value.content)
            if (is_text & is_image) | (is_image & is_keyval) | (is_keyval & is_text):
                rospy.logerr("Error happened when making widgetMarkup json. Please fill in one of the text_paragraph, image, key_value. Do not fill in more than two at the same time.")
            elif is_text:
                json_body.append({'textParagraph':msg.text_paragraph})
            elif is_image:
                image_json = {}
                if msg.image.image_uri:
                    image_json['imageUrl'] = msg.image.image_uri
                elif msg.image.localpath:
                    image_json['imageUrl'] = self._upload_file(msg.image.localpath)
                image_json['onClick'] = self._make_on_click_json(msg.image.on_click)
                image_json['aspectRatio'] = msg.image.aspect_ratio
                json_body.append(image_json)
            elif is_keyval:
                keyval_json = {}
                keyval_json['topLabel'] = msg.key_value.top_label
                keyval_json['content'] = msg.key_value.content
                keyval_json['contentMultiline'] = msg.key_value.content_multiline
                keyval_json['bottomLabel'] = msg.key_value.bottom_label
                keyval_json['onClick'] = self._make_on_click_json(msg.key_value.on_click)
                if msg.key_value.icon:
                    keyval_json['icon'] = msg.key_value.icon
                elif msg.key_value.original_icon_url:
                    keyval_json['iconUrl'] = msg.key_value.original_icon_url
                elif msg.key_value.original_icon_localpath:
                    keyval_json['iconUrl'] = self._upload_file(msg.key_value.original_icon_localpath)
                keyval_json['button'] = self._make_button_json(msg.key_value.button)
                json_body.append(keyval_json)
        return json_body

    def _make_on_click_json(self, on_click_msg):
        """Make onClick json.
        See https://developers.google.com/chat/api/reference/rest/v1/cards#onclick for details.
        :rtype on_click_msg: google_chat_ros.msg/OnClick.msg
        :rtype json_body: json
        """
        json_body = {}
        if on_click_msg.action.action_method_name and on_click_msg.open_link_url:
            rospy.logerr("Error happened when making onClick json. Please fill in one of the action, open_link_url. Do not fill in more than two at the same time.")
        elif on_click_msg.action.action_method_name:
            action = {}
            action['actionMethodName'] = on_click_msg.action.action_method_name
            parameters = []
            for parameter in on_click_msg.action.parameters:
                parameters.append({parameter['key']: parameter['value']})
            action['parameters'] = parameters
            json_body['action'] = action
        elif on_click_msg.open_link_url:
            json_body['openLink'] = on_click_msg.open_link_url
        return json_body

    def _make_button_json(self, button_msg):
        """Make button json.
        See https://developers.google.com/chat/api/reference/rest/v1/cards#button for details.
        :rtype button_msg: google_chat_ros.msg/Button.msg
        :rtype json_body: json
        """
        json_body = {}
        if button_msg.text_button_name and button_msg.image_button_name:
            rospy.logerr("Error happened when making Button json. Please fill in one of the text_button_name or image_button_name. Do not fill in more than two at the same time.")
        elif button_msg.text_button_name:
            rospy.loginfo("Build text button:{}".format(button_msg.text_button_name))
            text_button = {}
            text_button['text'] = button_msg.text_button_name
            text_button['onClick'] = self._make_on_click_json(button_msg.text_button_on_click)
            json_body['textButton'] = text_button
        elif button_msg.image_button_name:
            rospy.loginfo("Build image button:{}".format(button_msg.image_button_name))
            image_button = {}
            image_button['onClick'] = self._make_on_click_json(button_msg.image_button_on_click)
            if button_msg.icon:
                image_icon['icon'] = button_msg.icon
            elif button_msg.original_icon_url:
                image_icon['iconUrl'] = button_msg.original_icon_url
            elif button_msg.original_icon_filepath:
                image_icon['iconUrl'] = self._upload_file(button_msg.original_icon_filepath)
        return json_body

    def _get_user_info(self, item):
        user = User()
        user.name = item.get('name', '')
        user.display_name = item.get('displayName', '')
        user.avatar_url = item.get('avatarUrl', '')
        if self.download_avatar:
            user.avatar = self._get_image_from_uri(user.avatar_url)
        user.email = item.get('email', '')
        user.bot = True if item.get('type') == "BOT" else False
        user.human = True if item.get('type') == "HUMAN" else False
        return user

    def _upload_file(self, filepath):
        """Get local filepath and upload to Google Drive
        :param filepath: local file's path you want to upload
        :type filepath: string
        :returns: URL file exists
        :rtype: string
        """
        # ROS service client
        try:
            rospy.wait_for_service("~upload", timeout=5.0)
            gdrive_upload = rospy.ServiceProxy("~upload", Upload)
        except rospy.ROSException as e:
            rospy.logerr("No Google Drive ROS upload service was found. Please check gdrive_ros is correctly launched and service name is correct.")
            return
        # upload
        try:
            res = gdrive_upload(filepath)
        except rospy.ServiceException as e:
            rospy.logerr("Failed to call Google Drive upload service, status:{}".format(str(e)))
        else:
            url = res.file_url
            return url

    def _get_attachment(self, item):
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
        if self.download_data and attachment.download_uri:
            self._download_content(uri=attachment.download_uri, filename=attachment.content_name)
        if self.download_data and attachment.drive_field_id:
            self._download_content(drive_id=attachment.drive_field_id)
        return attachment

    def _get_image_from_uri(self, uri):
        try:
            img = requests.get(uri, timeout=10).content
        except Timeout:
            rospy.logerr("Exceeded timeout when downloading {}.".format(uri))
        except Exception as e:
            rospy.logwarn("Failed to get image from {}".format(uri))
            rospy.logerr(e)
        else:
            return img

    def _download_content(self, uri=None, drive_id=None, filename=''):
        try:
            if drive_id:
                path = os.path.join(self.download_directory, drive_id)
                gdown.download(id=drive_id, output=path)
            elif uri:
                path = os.path.join(self.download_directory, filename)
                gdown.download(url=uri, output=path)
        except Exception as e:
            rospy.logwarn("Failed to download the attatched file.")
            rospy.logerr(e)
        else:
            rospy.loginfo("Suceeded in downloading the attached file. Saved at {}".format(path))
        finally:
            return path

if __name__ == '__main__':
    rospy.init_node('google_chat', disable_signals=True)
    node = GoogleChatROS()
    rospy.spin()
