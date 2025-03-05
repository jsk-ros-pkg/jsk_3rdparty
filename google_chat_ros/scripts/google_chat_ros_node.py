#!/usr/bin/env python3
import datetime
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
from gdrive_ros.srv import *
from google_chat_ros.google_chat import GoogleChatRESTClient
from google_chat_ros.google_chat import GoogleChatHTTPSServer
from google_chat_ros.google_chat import GoogleChatPubSubClient
from google_chat_ros.msg import *
import rospy


class GoogleChatROS(object):
    """
    Send request to Google Chat REST API via ROS
    """
    def __init__(self):
        # rosparams
        receiving_chat_mode = rospy.get_param('~receiving_mode') # select from 'url', 'pubsub', 'none'
        google_credentials = rospy.get_param('~google_cloud_credentials_json')
        self.gdrive_ros_srv = rospy.get_param('~gdrive_upload_service', "/gdrive_ros/upload")
        self.upload_data_timeout = rospy.get_param('~upload_data_timeout', 30)
        self.download_data = rospy.get_param('~download_data', True)
        self.download_directory = rospy.get_param('~download_directory', "/tmp")
        self.download_avatar = rospy.get_param('~download_avatar', False)
        self.upload_parents_path = rospy.get_param(
            '~upload_parents_path', 'chat_notification')

        # For sending message
        self._client = GoogleChatRESTClient(google_credentials)
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

        # For receiving message
        if receiving_chat_mode in ("url", "dialogflow", "pubsub"):
            # ROS publisher
            self._message_activity_pub = rospy.Publisher("~message_activity", MessageEvent, queue_size=1)
            self._space_activity_pub = rospy.Publisher("~space_activity", SpaceEvent, queue_size=1)
            self._card_activity_pub = rospy.Publisher("~card_activity", CardEvent, queue_size=1)

            if receiving_chat_mode == "url":
                rospy.loginfo("Expected to get Google Chat Bot URL request")
                rospy.loginfo("Starting Google Chat HTTPS server...")
                self.host = rospy.get_param('~host')
                self.port = int(rospy.get_param('~port'))
                self.ssl_certfile = rospy.get_param('~ssl_certfile')
                self.ssl_keyfile = rospy.get_param('~ssl_keyfile')
                try:
                    self._server = GoogleChatHTTPSServer(
                        self.host, self.port, self.ssl_certfile, self.ssl_keyfile, callback=self.event_cb, user_agent='Google-Dynamite')
                    rospy.on_shutdown(self.killhttpd) # shutdown https server TODO is this okay in try ?
                    self._server.run()
                except ConnectionError as e:
                    rospy.logwarn("The error occurred while starting HTTPS server")
                    rospy.logerr(e)
            elif receiving_chat_mode == "pubsub":
                rospy.loginfo("Expected to use Google Cloud Pub Sub service")
                self.project_id = rospy.get_param("~project_id")
                self.subscription_id = rospy.get_param("~subscription_id")
                rospy.loginfo("      project_id : {}".format(self.project_id))
                rospy.loginfo(" subscription_id : {}".format(self.subscription_id ))
                self._pubsub_client = GoogleChatPubSubClient(
                    self.project_id, self.subscription_id, self.event_cb, google_credentials)
                rospy.on_shutdown(self.killpubsub)
                self._pubsub_client.run()

        elif receiving_chat_mode == "none":
            rospy.logwarn("You cannot recieve Google Chat event because HTTPS server or Google Cloud Pub/Sub is not running.")

        else:
            rospy.logerr("Please choose receiving_mode param from dialogflow, url, pubsub, none.")

    def killhttpd(self):
        self._server.kill()

    def killpubsub(self):
        self._pubsub_client.kill()

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

        # Card
        json_body['cards'] = []
        timestamp = '{0:%Y%m%d%H%M%S}'.format(datetime.datetime.now())
        if goal.cards:
            if goal.update_message:
                json_body['actionResponse'] = {"type": "UPDATE_MESSAGE"}
            for card in goal.cards:
                card_body = {}
                # card/header
                if card.header:
                    header = {}
                    header['title'] = card.header.title
                    header['subtitle'] = card.header.subtitle
                    header['imageStyle'] = 'AVATAR' if card.header.image_style_circular else 'IMAGE'
                    card_body['header'] = header
                    if card.header.image_url:
                        header['imageUrl'] = card.header.image_url
                    elif card.header.image_filepath:
                        header['imageUrl'] = self._upload_file(
                            card.header.image_filepath, timestamp=timestamp)
                # card/sections
                sections = []
                sections = self._make_sections_json(
                    card.sections, timestamp=timestamp)
                # card/actions
                card_actions = []
                for card_action_msg in card.card_actions:
                    card_action = {}
                    card_action['actionLabel'] = card_action_msg.action_label
                    card_action['onClick'] = self._make_on_click_json(card_action_msg.on_click)
                    card_actions.append(card_action)
                card_body['sections'] = sections
                if card_actions:
                    card_body['cardActions'] = card_actions
                card_body['name'] = card.name
                json_body['cards'].append(card_body)

        try:
            rospy.logdebug("Send json")
            rospy.logdebug(str(json_body))
            self._client.build_service()
            res = self._client.message_create(
                space=goal.space,
                json_body=json_body
            )
            result.message_result = self._make_message_msg({'message': res})
            # TODO add the result of what card was sent
            # result.cards_result =
        except Exception as e:
            rospy.logerr(str(e))
            feedback.status = str(e)
            success = False
        finally:
            self._as.publish_feedback(feedback)
            r.sleep()
            result.done = success
            self._as.set_succeeded(result)

    def event_cb(self, event, publish_topic=True):
        """Parse Google Chat API json content and publish as a ROS Message.
        See https://developers.google.com/chat/api/reference/rest 
        to check what contents are included in the json.
        :param event: A google Chat API POST request json content. 
        See https://developers.google.com/chat/api/guides/message-formats/events#event_fields for details.
        :rtype: ros message
        """
        rospy.logdebug("GOOGLE CHAT ORIGINAL JSON EVENT")
        rospy.logdebug(json.dumps(event, indent=2))
        # GET EVENT TYPE
        # event/eventTime
        event_time = event.get('eventTime', '')
        # event/space
        space = Space()
        space.name = event.get('space', {}).get('name', '')
        space.room = True if event.get('space', {}).get('type', '') == "ROOM" else False
        if space.room:
            space.display_name = event.get('space', {}).get('displayName', '')
        space.dm = True if event.get('space', {}).get('type', '') == "DM" else False
        # event/user
        user = self._get_user_info(event.get('user', {}))

        if event['type'] == 'ADDED_TO_SPACE' or event['type'] == 'REMOVED_FROM_SPACE':
            msg = SpaceEvent()
            msg.event_time = event_time
            msg.space = space
            msg.user = user
            msg.added = True if event['type'] == "ADDED_TO_SPACE" else False
            msg.removed = True if event['type'] == "REMOVED_FROM_SPACE" else False
            if publish_topic:
                self._space_activity_pub.publish(msg)
            return msg

        elif event['type'] == 'MESSAGE':
            msg = MessageEvent()
            msg.event_time = event_time
            msg.space = space
            msg.user = user
            msg.message = self._make_message_msg(event)
            if publish_topic:
                self._message_activity_pub.publish(msg)
            return msg

        elif event['type'] == 'CARD_CLICKED':
            msg = CardEvent()
            msg.event_time = event_time
            msg.space = space
            msg.user = user
            msg.message = self._make_message_msg(event)
            if event.get('action'):
                action = event.get('action')
                msg.action.action_method_name = action.get('actionMethodName')
                if action.get('parameters'):
                    for param in action.get('parameters'):
                        action_parameter = ActionParameter()
                        action_parameter.key = param.get("key") if param.get("key") else ""
                        action_parameter.value = param.get("value") if param.get("value") else ""
                        msg.action.parameters.append(action_parameter)
            if publish_topic:
                self._card_activity_pub.publish(msg)
            return msg

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

    def _make_sections_json(self, sections_msg, timestamp=None):
        """
        :type msg: list of google_chat_ros.msgs/Section
        :rtype json_body: list of json
        """
        json_body = []
        for msg in sections_msg:
            section = {}
            if msg.header:
                section['header'] = msg.header
            section['widgets'] = self._make_widget_markups_json(
                msg.widgets, timestamp=timestamp)
            json_body.append(section)
        return json_body

    def _make_widget_markups_json(self, widgets_msg, timestamp=None):
        """Make widget markup json lists.
        See https://developers.google.com/chat/api/reference/rest/v1/cards#widgetmarkup for details.
        :rtype widgets_msg: list of google_chat_ros.msgs/WidgetMarkup
        :rtype json_body: list of json
        """
        json_body = []
        for msg in widgets_msg:
            is_text = bool(msg.text_paragraph)
            is_image = bool(msg.image.image_url) or bool(msg.image.localpath)
            is_keyval = bool(msg.key_value.content)
            # make buttons
            buttons = []
            buttons_msg = msg.buttons
            for button_msg in buttons_msg:
                buttons.append(
                    self._make_button_json(button_msg, timestamp=timestamp))
            if buttons:
                json_body.append({'buttons':buttons})

            if (is_text & is_image) | (is_image & is_keyval) | (is_keyval & is_text):
                rospy.logerr("Error happened when making widgetMarkup json. Please fill in one of the text_paragraph, image, key_value. Do not fill in more than two at the same time.")
            elif is_text:
                json_body.append({'textParagraph':{'text':msg.text_paragraph}})
            elif is_image:
                image_json = {}
                if msg.image.image_url:
                    image_json['imageUrl'] = msg.image.image_url
                elif msg.image.localpath:
                    image_json['imageUrl'] = self._upload_file(
                        msg.image.localpath, timestamp=timestamp)
                if msg.image.on_click.action.action_method_name or msg.image.on_click.open_link_url:
                    image_json['onClick'] = self._make_on_click_json(msg.image.on_click)
                if msg.image.aspect_ratio:
                    image_json['aspectRatio'] = msg.image.aspect_ratio
                json_body.append({'image':image_json})
            elif is_keyval:
                keyval_json = {}
                keyval_json['topLabel'] = msg.key_value.top_label
                keyval_json['content'] = msg.key_value.content
                keyval_json['contentMultiline'] = msg.key_value.content_multiline
                keyval_json['bottomLabel'] = msg.key_value.bottom_label
                if msg.key_value.on_click.action.action_method_name or msg.key_value.on_click.open_link_url:
                    keyval_json['onClick'] = self._make_on_click_json(msg.key_value.on_click)
                if msg.key_value.icon:
                    keyval_json['icon'] = msg.key_value.icon
                elif msg.key_value.original_icon_url:
                    keyval_json['iconUrl'] = msg.key_value.original_icon_url
                elif msg.key_value.original_icon_localpath:
                    keyval_json['iconUrl'] = self._upload_file(
                        msg.key_value.original_icon_localpath,
                        timestamp=timestamp)
                if msg.key_value.button.text_button_name or msg.key_value.button.image_button_name:
                    keyval_json['button'] = self._make_button_json(
                        msg.key_value.button, timestamp=timestamp)
                json_body.append({'keyValue':keyval_json})
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
                parameters.append({'key':parameter.key, 'value':parameter.value})
            action['parameters'] = parameters
            json_body['action'] = action
        elif on_click_msg.open_link_url:
            json_body['openLink'] = {'url': on_click_msg.open_link_url}
        return json_body

    def _make_button_json(self, button_msg, timestamp=None):
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
                image_icon['iconUrl'] = self._upload_file(
                    button_msg.original_icon_filepath, timestamp=timestamp)
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

    def _upload_file(self, filepath, return_id=False, timestamp=None):
        """Get local filepath and upload to Google Drive
        :param filepath: local file's path you want to upload
        :type filepath: string
        :returns: URL file exists
        :rtype: string
        """
        # ROS service client
        try:
            rospy.wait_for_service(self.gdrive_ros_srv, timeout=self.upload_data_timeout)
            gdrive_upload = rospy.ServiceProxy(self.gdrive_ros_srv, Upload)
        except rospy.ROSException as e:
            rospy.logerr("No Google Drive ROS upload service was found. Please check gdrive_ros is correctly launched and service name is correct.")
            rospy.logerr(e)
            return
        # upload
        if timestamp is None:
            parents_path = self.upload_parents_path
        else:
            parents_path = '{}/{}'.format(
                self.upload_parents_path, timestamp)
        try:
            res = gdrive_upload(
                file_path=filepath,
                parents_path=parents_path,
                use_timestamp_folder=False,
            )
        except rospy.ServiceException as e:
            rospy.logerr("Failed to call Google Drive upload service, status:{}".format(str(e)))
        else:
            if return_id:
                drive_id = res.file_id
                rospy.loginfo("Google drive ID:{}".format(drive_id))
                return drive_id
            else:
                url = res.file_url
                rospy.loginfo("Google drive URL:{}".format(url))
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
    rospy.init_node('google_chat')
    node = GoogleChatROS()
    rospy.spin()
