#!/usr/bin/env python3
import datetime
import gdown
import json
import os
import requests
from requests.exceptions import Timeout
from requests.exceptions import ConnectionError

# ROS libraries
import ast
try:
    from dialogflow_task_executive.msg import DialogResponse
except ImportError:
    # dialogflow_task_executive is not ported to ROS2 yet; only used by
    # dialogflow_cb() (receiving_mode: dialogflow)
    DialogResponse = None
from google_chat_ros.google_chat import GoogleChatRESTClient
from google_chat_ros.google_chat import GoogleChatHTTPSServer
from google_chat_ros.google_chat import GoogleChatPubSubClient
from google_chat_ros.msg import *

try:
    from jsk_ros1_ros2_compat import log_utils
    from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat
    from jsk_ros1_ros2_compat.rospy_rclpy_compat import ROS_VERSION
except ImportError:
    # jsk_common < 3.0 has no jsk_ros1_ros2_compat, so this is ROS1 only:
    # forward ros_compat.xxx(node, ...) to rospy.xxx(...)
    import rospy
    ROS_VERSION = 1
    import actionlib
    class log_utils(object):
        set_node = staticmethod(lambda node: None)
    class ros_compat(object):
        create_publisher = staticmethod(lambda node, *args, **kwargs: rospy.Publisher(*args, **kwargs))
        create_simple_action_server = staticmethod(lambda node, name, action_type, execute_cb: actionlib.SimpleActionServer(name, action_type, execute_cb, auto_start=False))
        get_param = staticmethod(lambda node, *args, **kwargs: rospy.get_param(*args, **kwargs))
        init_node = staticmethod(lambda name, **kwargs: rospy.init_node(name, **kwargs))
        spin = staticmethod(lambda node, executor=None: rospy.spin())
        logdebug = staticmethod(lambda node, *args, **kwargs: rospy.logdebug(*args, **kwargs))
        loginfo = staticmethod(lambda node, *args, **kwargs: rospy.loginfo(*args, **kwargs))
        logwarn = staticmethod(lambda node, *args, **kwargs: rospy.logwarn(*args, **kwargs))
        logerr = staticmethod(lambda node, *args, **kwargs: rospy.logerr(*args, **kwargs))
        on_shutdown = staticmethod(lambda node, callback: rospy.on_shutdown(callback))
        Rate = staticmethod(lambda node, hz: rospy.Rate(hz))
if ROS_VERSION == 2:
    import threading
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from google_chat_ros.action import SendMessage as SendMessageAction
    from gdrive_ros.srv import Upload
    SendMessageResult = SendMessageAction.Result
    SendMessageFeedback = SendMessageAction.Feedback
    # rest_cb() waits for the gdrive_ros upload service from an executor
    # callback (see _ros2_upload_file())
    make_executor = MultiThreadedExecutor
else:
    import threading
    import actionlib
    import rospy
    from gdrive_ros.srv import *
    make_executor = lambda: None


class GoogleChatROS(object):
    """
    Send request to Google Chat REST API via ROS
    """
    def __init__(self, node=None):
        self.node = node
        # rosparams
        receiving_chat_mode = ros_compat.get_param(node, '~receiving_mode') # select from 'url', 'pubsub', 'none'
        google_credentials = ros_compat.get_param(node, '~google_cloud_credentials_json')
        self.gdrive_ros_srv = ros_compat.get_param(node, '~gdrive_upload_service', "/gdrive_ros/upload")
        self.upload_data_timeout = ros_compat.get_param(node, '~upload_data_timeout', 30)
        self.download_data = ros_compat.get_param(node, '~download_data', True)
        self.download_directory = ros_compat.get_param(node, '~download_directory', "/tmp")
        self.download_avatar = ros_compat.get_param(node, '~download_avatar', False)
        self.upload_parents_path = ros_compat.get_param(
            node, '~upload_parents_path', 'chat_notification')

        # For sending message
        self._client = GoogleChatRESTClient(google_credentials)
        ros_compat.loginfo(self.node, "Starting Google Chat REST service...")
        try:
            self._client.build_service() # Start google chat authentication and service
            ros_compat.loginfo(self.node, "Succeeded in starting Google Chat REST service")
            # ROS ActionLib
            self._as = ros_compat.create_simple_action_server(
                node, '~send', SendMessageAction,
                self.rest_cb)
            self._as.start()
        except Exception as e:
            ros_compat.logwarn(self.node, "Failed to start Google Chat REST service")
            ros_compat.logerr(self.node, str(e))

        # For receiving message
        if receiving_chat_mode in ("url", "dialogflow", "pubsub"):
            # ROS publisher
            self._message_activity_pub = ros_compat.create_publisher(node, "~message_activity", MessageEvent, queue_size=1)
            self._space_activity_pub = ros_compat.create_publisher(node, "~space_activity", SpaceEvent, queue_size=1)
            self._card_activity_pub = ros_compat.create_publisher(node, "~card_activity", CardEvent, queue_size=1)

            if receiving_chat_mode == "url":
                ros_compat.loginfo(self.node, "Expected to get Google Chat Bot URL request")
                ros_compat.loginfo(self.node, "Starting Google Chat HTTPS server...")
                self.host = ros_compat.get_param(node, '~host')
                self.port = int(ros_compat.get_param(node, '~port'))
                self.ssl_certfile = ros_compat.get_param(node, '~ssl_certfile')
                self.ssl_keyfile = ros_compat.get_param(node, '~ssl_keyfile')
                try:
                    self._server = GoogleChatHTTPSServer(
                        self.host, self.port, self.ssl_certfile, self.ssl_keyfile, callback=self.event_cb, user_agent='Google-Dynamite')
                    ros_compat.on_shutdown(self.node, self.killhttpd) # shutdown https server TODO is this okay in try ?
                    self._server.run()
                except ConnectionError as e:
                    ros_compat.logwarn(self.node, "The error occurred while starting HTTPS server")
                    ros_compat.logerr(self.node, str(e))
            elif receiving_chat_mode == "pubsub":
                ros_compat.loginfo(self.node, "Expected to use Google Cloud Pub Sub service")
                self.project_id = ros_compat.get_param(node, "~project_id")
                self.subscription_id = ros_compat.get_param(node, "~subscription_id")
                ros_compat.loginfo(self.node, "      project_id : {}".format(self.project_id))
                ros_compat.loginfo(self.node, " subscription_id : {}".format(self.subscription_id ))
                self._pubsub_client = GoogleChatPubSubClient(
                    self.project_id, self.subscription_id, self.event_cb, google_credentials)
                ros_compat.on_shutdown(self.node, self.killpubsub)
                self._pubsub_client.run()

        elif receiving_chat_mode == "none":
            ros_compat.logwarn(self.node, "You cannot recieve Google Chat event because HTTPS server or Google Cloud Pub/Sub is not running.")

        else:
            ros_compat.logerr(self.node, "Please choose receiving_mode param from dialogflow, url, pubsub, none.")

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
        r = ros_compat.Rate(self.node, 1)
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
            ros_compat.logdebug(self.node, "Send json")
            ros_compat.logdebug(self.node, str(json_body))
            self._client.build_service()
            res = self._client.message_create(
                space=goal.space,
                json_body=json_body
            )
            result.message_result = self._make_message_msg({'message': res})
            # TODO add the result of what card was sent
            # result.cards_result =
        except Exception as e:
            ros_compat.logerr(self.node, str(e))
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
        ros_compat.logdebug(self.node, "GOOGLE CHAT ORIGINAL JSON EVENT")
        ros_compat.logdebug(self.node, json.dumps(event, indent=2))
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
            ros_compat.logerr(self.node, "Got unknown event type.")
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
                ros_compat.logerr(self.node, "Error happened when making widgetMarkup json. Please fill in one of the text_paragraph, image, key_value. Do not fill in more than two at the same time.")
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
            ros_compat.logerr(self.node, "Error happened when making onClick json. Please fill in one of the action, open_link_url. Do not fill in more than two at the same time.")
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
            ros_compat.logerr(self.node, "Error happened when making Button json. Please fill in one of the text_button_name or image_button_name. Do not fill in more than two at the same time.")
        elif button_msg.text_button_name:
            ros_compat.loginfo(self.node, "Build text button:{}".format(button_msg.text_button_name))
            text_button = {}
            text_button['text'] = button_msg.text_button_name
            text_button['onClick'] = self._make_on_click_json(button_msg.text_button_on_click)
            json_body['textButton'] = text_button
        elif button_msg.image_button_name:
            ros_compat.loginfo(self.node, "Build image button:{}".format(button_msg.image_button_name))
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
        if ROS_VERSION == 2:
            return self._ros2_upload_file(filepath, return_id, timestamp)
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

    def _ros2_upload_file(self, filepath, return_id=False, timestamp=None):
        """ROS2 version of _upload_file(). This runs in rest_cb(), i.e. in an
        executor callback, so the service response is waited for with a
        reentrant callback group instead of ros_compat.call_service(),
        which spins the node itself."""
        if getattr(self, '_gdrive_upload', None) is None:
            self._gdrive_upload = self.node.create_client(
                Upload, self.gdrive_ros_srv,
                callback_group=ReentrantCallbackGroup())
        if not self._gdrive_upload.wait_for_service(
                timeout_sec=self.upload_data_timeout):
            self.node.get_logger().error("No Google Drive ROS upload service was found. Please check gdrive_ros is correctly launched and service name is correct.")
            return
        # upload
        if timestamp is None:
            parents_path = self.upload_parents_path
        else:
            parents_path = '{}/{}'.format(
                self.upload_parents_path, timestamp)
        done = threading.Event()
        future = self._gdrive_upload.call_async(Upload.Request(
            file_path=filepath,
            parents_path=parents_path,
            use_timestamp_folder=False,
        ))
        future.add_done_callback(lambda _: done.set())
        if not done.wait(self.upload_data_timeout) or future.result() is None:
            self.node.get_logger().error("Failed to call Google Drive upload service")
            return
        res = future.result()
        if return_id:
            self.node.get_logger().info("Google drive ID:{}".format(res.file_id))
            return res.file_id
        else:
            self.node.get_logger().info("Google drive URL:{}".format(res.file_url))
            return res.file_url

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
            ros_compat.logerr(self.node, "Exceeded timeout when downloading {}.".format(uri))
        except Exception as e:
            ros_compat.logwarn(self.node, "Failed to get image from {}".format(uri))
            ros_compat.logerr(self.node, str(e))
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
            ros_compat.logwarn(self.node, "Failed to download the attatched file.")
            ros_compat.logerr(self.node, str(e))
        else:
            ros_compat.loginfo(self.node, "Suceeded in downloading the attached file. Saved at {}".format(path))
        finally:
            return path

if __name__ == '__main__':
    node = ros_compat.init_node('google_chat')
    log_utils.set_node(node)
    # GoogleChatROS() blocks in url/pubsub receiving mode, so run it in a
    # daemon thread and spin here: rclpy handles SIGINT/SIGTERM only when
    # spinning in the main thread, and the receiving server stops with the
    # process.
    app_thread = threading.Thread(target=GoogleChatROS, args=(node,))
    app_thread.daemon = True
    app_thread.start()
    ros_compat.spin(node, make_executor())
