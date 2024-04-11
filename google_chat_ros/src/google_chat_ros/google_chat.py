from apiclient.discovery import build
import base64
from concurrent.futures import TimeoutError
from google.cloud import pubsub_v1
from google.oauth2.service_account import Credentials
from httplib2 import Http
import http.server as s
import json
from oauth2client.service_account import ServiceAccountCredentials
import rospy # for logging
import socket
import ssl

class GoogleChatRESTClient():
    def __init__(self, keyfile):
        self._auth_scopes = "https://www.googleapis.com/auth/chat.bot"
        self.keyfile = keyfile
        self.__credentials = None
        self._chat = None

    def build_service(self):
        """Authenticate Googel REST API and start service by json key file.
        Please see https://developers.google.com/chat/how-tos/service-accounts#step_1_create_service_account_and_private_key for details.
        :param keyfile_path: str, the file path of json key file
        """
        self.__credentials = ServiceAccountCredentials.from_json_keyfile_name(self.keyfile, self._auth_scopes)
        self._chat = build('chat', 'v1', http=self.__credentials.authorize(Http()))

    def message_create(self, space, json_body):
        if not space.startswith('spaces/'):
            raise RuntimeError("Space name must begin with spaces/")
        # returns same 403 error both authenticate error and not connected error
        return self._chat.spaces().messages().create(parent=space, body=json_body).execute()

    def list_members(self, space):
        """Show member list in the space.
        """
        if not space.startswith('spaces/'):
            raise RuntimeError("Space name must begin with spaces/")
        return self._chat.spaces().members().list(parent=space).execute()

class GoogleChatHTTPSServer():
    """The server for getting https request from Google Chat
    """
    def __init__(self, host, port, certfile, keyfile, callback, user_agent):
        """
        :param host: str, hostname
        :param port: int, port number
        :param certfile: str, ssl certfile path
        :param keyfile: str, ssl keyfile path
        """
        self._host = host
        self._port = port
        self._certfile = certfile
        self._keyfile = keyfile
        self._callback = callback
        self.user_agent = user_agent
        self.__RUN = True

    def __handler(self, *args):
        try:
            GoogleChatHTTPSHandler(self._callback, self.user_agent, *args)
        except socket.error as e:
            if e.errno == 104: # ignore SSL Connection reset error
                rospy.logdebug(e)
            else:
                raise e

    def run(self):
        self._httpd = s.HTTPServer((self._host, self._port), self.__handler)
        self._httpd.socket = ssl.wrap_socket(self._httpd.socket, certfile=self._certfile, keyfile=self._keyfile)
        while self.__RUN:
            self._httpd.handle_request()

    def kill(self):
        self.__RUN = False
        self._httpd.server_close()

class GoogleChatHTTPSHandler(s.BaseHTTPRequestHandler):
    """The handler for https request from Google chat API. Mainly used for receiving messages, events.
    """
    def __init__(self, callback, user_agent, *args):
        self._callback = callback
        self.user_agent = user_agent
        s.BaseHTTPRequestHandler.__init__(self, *args)

    def do_POST(self):
        """Handles an event from Google Chat.
        Please see https://developers.google.com/chat/api/guides/message-formats/events for details.
        """
        user_agent = self.headers.get("User-Agent")
        print('user_agent ' + str(user_agent))
        if user_agent == self.user_agent:
            self._parse_json()
            self._callback(self.json_content)
            self._response()

    def _parse_json(self):
        content_len = int(self.headers.get("content-length"))
        request_body = self.rfile.read(content_len).decode('utf-8')
        self.json_content = json.loads(request_body) # json.loads returns unicode by default

    def _response(self):
        self.send_response(200)
        self.end_headers()

    def _bad_request(self):
        self.send_response(400)
        self.end_headers()

class GoogleChatPubSubClient():
    def __init__(self, project_id, subscription_id, callback, keyfile):
        self._callback = callback
        self.__credentials = Credentials.from_service_account_file(keyfile)
        self._sub = pubsub_v1.SubscriberClient(credentials=self.__credentials)
        sub_path = self._sub.subscription_path(project_id, subscription_id)
        self._streaming_pull_future = self._sub.subscribe(sub_path, callback=self._pubsub_cb)

    def _pubsub_cb(self, message):
        rospy.logdebug("Recieved {message}")
        rospy.logdebug(message.data)
        try:
            json_content = json.loads(message.data)
            self._callback(json_content)
        except Exception as e:
            rospy.logerr("Failed to handle the request from Cloud PubSub.")
            rospy.logerr("It might be caused because of invalid type message from GCP")
            rospy.logerr(e)
        finally:
            message.ack()

    def run(self):
        with self._sub:
            try:
                self._streaming_pull_future.result()
            except KeyboardInterrupt:
                self._streaming_pull_future.cancel()

    def kill(self):
        self._streaming_pull_future.cancel()
