#!/usr/bin/env python
# -*- encoding: utf-8 -*-
import rospy
import rospkg
from dialogflow_task_executive.msg import DialogResponse
from dialogflow_webhook_ros.msg import OriginalDetectIntentRequest

import http.server as s
import os, json
import socket
import ssl


class Server(object):
    """
    This server is expected to accept the POST https message from Google DialogFlow.
    If the server received the message, it publishes the topic 'dialogflow_task_executive.msg DialogResponse' and send response to Google DialogFlow.
    """
    def __init__(self):
        rospy.init_node('~dialogflow_webhook_server', disable_signals=True)
        self.host = rospy.get_param('~host')
        self.port = int(rospy.get_param('~port'))
        self.ssl_certfile = rospy.get_param('~ssl_certfile')
        self.ssl_keyfile = rospy.get_param('~ssl_keyfile')
        rospy.on_shutdown(self.killnode)
        self._run_handler()

    def killnode(self):
        self.httpd.shutdown()

    def __handler(self, *args):
        try:
            DialogFlowHandler(*args)
        except socket.error as e:
            if e.errno == 104: # ignore SSL Connection reset error
                rospy.logdebug(e)
            else:
                raise e            

    def _run_handler(self):
        self.httpd = s.HTTPServer((self.host, self.port), self.__handler)
        self.httpd.socket = ssl.wrap_socket(self.httpd.socket, certfile=self.ssl_certfile, keyfile=self.ssl_keyfile, server_side=True)


class DialogFlowHandler(s.BaseHTTPRequestHandler):
    """
    The handler to react the POST from Google DialogFlow and publish a ROS topic.
    """
    def __init__(self, *args):
        self.dialogflow_pub = rospy.Publisher('dialog_response', DialogResponse, queue_size=1)
        self.application_request_pub = rospy.Publisher('dialogflow_original_application_request', OriginalDetectIntentRequest, queue_size=1)
        s.BaseHTTPRequestHandler.__init__(self, *args)

    def do_POST(self):
        """
        The Handler is expected to recieve POST method from Google Dialogflow. If the request is not the POST method or from Google Dialogflow, it returns the error.
        """
        user_agent = self.headers.get("User-Agent")
        rospy.loginfo('Recieved POST request' + str(self.headers))
        if user_agent == "Google-Dialogflow":
            self._parse_json()
            self._publish()
            self._response()
        else:
            rospy.logwarn('User-Agent header should be Google-Dialogflow, but got ' + user_agent)
            self._bad_request()

    def _parse_json(self):
        content_len = int(self.headers.get("content-length"))
        request_body = self.rfile.read(content_len).decode("utf-8")
        self.json_content = json.loads(request_body)

    def _publish(self):
        """
        Publish ROS message.
        """
        time_stamp = rospy.Time.now()
        # Dialogflow response query result
        dialogflow_msg = DialogResponse()
        dialogflow_msg.header.stamp = time_stamp
        dialogflow_msg.query = self.json_content.get('queryResult', {}).get('queryText', '')
        dialogflow_msg.action = self.json_content.get('queryResult', {}).get('action', '')
        dialogflow_msg.response = self.json_content.get('queryResult', {}).get('fulfillmentText', '')
        if self.json_content.get('queryResult', {}).get('allRequiredParamsPresent', '') == 'True':
            dialogflow_msg.fulfilled = True
        dialogflow_msg.parameters = json.dumps(self.json_content.get('queryResult', {}).get('parameters', ''))
        dialogflow_msg.speech_score = 1.0
        dialogflow_msg.intent_score = self.json_content.get('queryResult', {}).get('intentDetectionConfidence', 0.0)
        rospy.logdebug("The query from Dialogflow \n" + str(dialogflow_msg))
        self.dialogflow_pub.publish(dialogflow_msg)

        # Original Application request
        application_msg = OriginalDetectIntentRequest()
        application_msg.header.stamp = time_stamp
        application_msg.source = self.json_content.get('originalDetectIntentRequest', {}).get('source', '')
        application_msg.version = self.json_content.get('originalDetectIntentRequest', {}).get('version', '')
        application_msg.payload = str(self.json_content.get('originalDetectIntentRequest', {}).get('payload', ''))
        rospy.logdebug("The request from original application \n" + str(application_msg))
        self.application_request_pub.publish(application_msg)

    def _make_response(self):
        """
        This function is in development. If you want to develop the application like the robot sends its state to the Dialogflow and the Dialogflow handle it, you have to make new response_type. 
        Please see https://cloud.google.com/dialogflow/es/docs/fulfillment-webhook#webhook_response for details.
        """
        res_body = {}

        # default text response
        # res_body["fulfillmentMessages"] = []
        # res_body["fulfillmentMessages"].append({})
        # res_body["fulfillmentMessages"][0]["text"] = {"text": ["Text response from webhook"]}

        self.res_body = json.dumps(res_body, indent=2)

    def _response(self):
        """
        The DialogFlow client expects the non-empty response.
        Please see https://cloud.google.com/dialogflow/es/docs/fulfillment-webhook for details.
        """
        self._make_response()
        self.send_response(200)
        self.send_header('Content-type', 'application/json; charset=utf-8')
        self.send_header('Content-length', len(self.res_body.encode()))
        self.end_headers()
        self.wfile.write(self.res_body.encode('utf-8'))

    def _bad_request(self):
        self.send_response(400)
        self.end_headers()


if __name__ == '__main__':
    server = Server()
    rospy.loginfo('DialogFlow HTTPS Server starts - %s:%s' % (server.host, server.port))
    server.httpd.serve_forever()
