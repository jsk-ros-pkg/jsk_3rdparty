#!/usr/bin/env python
from __future__ import print_function

from apiclient.discovery import build
from googleapiclient.errors import HttpError
from httplib2 import Http
import json
from oauth2client.service_account import ServiceAccountCredentials

class GoogleChatClient():
    def __init__(self, keyfile):
        self._auth_scopes = "https://www.googleapis.com/auth/chat.bot"
        self.keyfile = keyfile
        self.__credentials = None
        self._chat = None

    def build_service(self):
        """Authenticate Google REST API and start service by json key file.
        Please see https://developers.google.com/chat/how-tos/service-accounts#step_1_create_service_account_and_private_key for details.
        :param keyfile_path: str, the file path of json key file
        """
        self.__credentials = ServiceAccountCredentials.from_json_keyfile_name(self.keyfile, self._auth_scopes)
        self._chat = build('chat', 'v1', http=self.__credentials.authorize(Http()))
        
    def send_text(self, space, text):
        parent = 'spaces/' + space
        return self._chat.spaces().messages().create(parent=parent, body={'text':text}).execute() # returns same 403 error both authenticate error and not connected error

    def send_card(self, space, content):
        pass
