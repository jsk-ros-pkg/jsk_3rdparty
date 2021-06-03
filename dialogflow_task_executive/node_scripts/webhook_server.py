#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import rospy
import rospkg
from dialogflow_task_executive.msg import DialogResponse

import http.server as s
from urllib.parse import urlparse, parse_qs
import ssl

import json


class Server(object):
    """
    This server is expected to accept the POST https message from Google DialogFlow.
    If the server received the message, it publishes the topic 'dialogflow_task_executive.msg DialogResponse' and send response to Google DialogFlow.
    """
    def __init__(self):
        """
        You need to set the path to certfile for ssl connection. You shouldn't use self-signed certificate.
        """
        rospack = rospkg.RosPack()
        conffile = rospack.get_path('dialogflow_task_executive') + "/config/webhook.json"
        with open(conffile) as f:
            json_dict = json.load(f)
            self.host = json_dict['host']
            self.port = json_dict['port']
            self._certfile_path = json_dict['certfile']
            self._keyfile_path = json_dict['keyfile']
        rospy.on_shutdown(self.killnode)
        rospy.init_node('dialogflow_webhook_server', disable_signals=True)
        self._run_handler()
        
    def killnode(self):
        self.httpd.shutdown()
        
    def _run_handler(self):
        self.httpd = s.HTTPServer((self.host, self.port), DialogFlowHandler)
        self.httpd.socket = ssl.wrap_socket(self.httpd.socket, certfile=self._certfile_path, keyfile=self._keyfile_path, server_side=True)

        
class DialogFlowHandler(s.BaseHTTPRequestHandler):
    """
    The handler to react the POST from Google DialogFlow and publish a ROS topic.
    """
    def __init__(self, *args):
        self.pub = rospy.Publisher('dialog_response', DialogResponse, queue_size=1)
        s.BaseHTTPRequestHandler.__init__(self, *args)
    
    def do_POST(self):
        """
        The Handler is expected to recieve POST method from Google Dialogflow. If the request is not the POST method or from Google Dialogflow, it returns the error.
        """
        user_agent = self.headers.get("User-Agent")
        rospy.loginfo('Recieved POST request' + str(self.headers))
        if user_agent == "Google-Dialogflow":
            self._parse_json()
            self._pub_task()
            self._response()
        else:
            rospy.logwarn('User-Agent header should be Google-Dialogflow, but got ' + user_agent)
            self._bad_request()

    def _parse_json(self):
        content_len = int(self.headers.get("content-length"))
        request_body = self.rfile.read(content_len).decode("utf-8")
        self.json_content = json.loads(request_body)

    def _pub_task(self):
        """
        Publish ROS message to exec task.
        """
        msg = DialogResponse()
        msg.header.stamp = rospy.Time.now()
        msg.query = self.json_content['queryResult']['queryText']
        msg.action = self.json_content['queryResult']['action']
        msg.response = self.json_content['queryResult']['fulfillmentText']
        if self.json_content['queryResult']['allRequiredParamsPresent'] == 'True':
            msg.fulfilled = True
        msg.parameters = json.dumps(self.json_content['queryResult']['parameters'])
        msg.speech_score = 1.0
        msg.intent_score = self.json_content['queryResult']['intentDetectionConfidence']
        rospy.loginfo("The message summary from Dialogflow \n" + str(msg))
        self.pub.publish(msg)

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
    try:
        server = Server()
        rospy.loginfo('DialogFlow HTTPS Server starts - %s:%s' % (server.host, server.port))
        server.httpd.serve_forever()
    except Exception as e:
        rospy.logerr(e)
