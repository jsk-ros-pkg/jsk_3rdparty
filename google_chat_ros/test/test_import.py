#!/usr/bin/env python
import unittest
import sys

PKG = 'google_chat_ros'
NAME = 'test_import'

class TestBlock(unittest.TestCase):
    def __init__(self, *args):
        super(TestBlock, self).__init__(*args)

    def test_import(self):
        try:
            from apiclient.discovery import build
            import base64
            from concurrent.futures import TimeoutError
            import gdown
            from google.cloud import pubsub_v1
            from google.oauth2.service_account import Credentials
            from httplib2 import Http
            import http.server as s
            import json
            from oauth2client.service_account import ServiceAccountCredentials
            import requests
            import socket
            import ssl
        except Exception as e:
            print(str(e))
            assert False

if __name__ == "__main__":
    import rostest
    rostest.rosrun(PKG, NAME, TestBlock, sys.argv)
