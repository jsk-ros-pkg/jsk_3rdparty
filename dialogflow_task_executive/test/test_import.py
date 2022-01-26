#!/usr/bin/env python
import unittest

PKG = 'dialogflow_task_executive'
NAME = 'test_import'

class TestBlock(unittest.TestCase):
    def __init__(self, *args):
        super(TestBlock, self).__init__(*args)

    def test_import(self):
        try:
            import dialogflow as df
            from google.oauth2.service_account import Credentials
            from google.protobuf.json_format import MessageToJson
        except Exception as e:
            assert False

if __name__ == "__main__":
    import rostest
    rostest.rosrun(PKG, NAME, TestBlock)
