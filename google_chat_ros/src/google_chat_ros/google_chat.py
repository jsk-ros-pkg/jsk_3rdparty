from __future__ import print_function

from apiclient.discovery import build
from flask import Flask, request, json
from httplib2 import Http
import http.server as s
from oauth2client.service_account import ServiceAccountCredentials
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

    def send_text(self, space, text):
        parent = 'spaces/' + space
        # returns same 403 error both authenticate error and not connected error
        return self._chat.spaces().messages().create(parent=parent, body={'text': text}).execute()

    def send_card(self, space, content):
        pass

    def list_members(self, space):
        """Show member list in the space.
        """
        parent = 'spaces/' + space
        return self._chat.spaces().members().list(parent=parent).execute()

class GoogleChatHTTPSServer():
    """The server for getting https request from Google Chat
    """
    def __init__(self, host, port, certfile, keyfile, callback):
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

    def run(self):
        self._httpd = s.HTTPServer((self._host, self._port), GoogleChatHTTPSHandler(callback=self._callback))
        self._httpd.socket = ssl.wrap_socket(self._httpd.socket, certfile=self._certfile, keyfile=self._keyfile)

    def kill(self):
        self._httpd.shutdown()

class GoogleChatHTTPSHandler(s.BaseHTTPRequestHandler):
    """The handler for https request from Google chat API. Mainly used for recieving messages, events.
    """
    def __init__(self, callback, *args):
        s.BaseHTTPRequestHandler.__init__(self, *args)
        self._callback = callback

    def do_POST(self):
        """Handles an event from Google Chat.
        Please see https://developers.google.com/chat/api/guides/message-formats/events for details.
        """
        user_agent = self.headers.get("User-Agent")
        self._parse_json()
        self._callback(json)
        self._response()

    def _parse_json(self):
        content_len = int(self.headers.get("content-length"))
        request_body = self.rfile.read(content_len).decode("utf-8")
        self.json_content = json.loads(request_body)

    def _response(self):
        self.send_response(200)
        self.send_header('Content-type', 'application/json; charset=utf-8')
        self.send_header('Content-length', len(self.res_body.encode()))
        self.end_headers()
        self.wfile.write(self.res_body.encode('utf-8'))

    def _bad_request(self):
        self.send_response(400)
        self.end_headers()

