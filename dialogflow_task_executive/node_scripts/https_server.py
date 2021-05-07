#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
import rospy
import rospkg
from dialogflow_task_executive.msg import DialogResponse
import http.server as s
from urllib.parse import urlparse, parse_qs
import ssl
import logging
import json

class Server():
    def __init__(self):
        rospack = rospkg.RosPack()
        httpsconffile = rospack.get_path('dialogflow_task_executive') + "/config/https.json"
        certfile = rospack.get_path('dialogflow_task_executive') + "/auth/certfile.json"
        with open(httpsconffile) as f:
            json_dict = json.load(f)
            self.host = json_dict['ip']
            self.port = int(json_dict['port'])
        with open(certfile) as f:
            json_dict = json.load(f)
            self._certfile_path = json_dict['certfile']
            self._keyfile_path = json_dict['keyfile']
        self._run_handler()
        rospy.init_node('dialogflow_https_server')
        rospy.on_shutdown(self.killnode)
        
    def killnode(self):
        self.httpd.shutdown()
        
    def _run_handler(self):
        self.httpd = s.HTTPServer((self.host, self.port), DialogFlowHandler)
        self.httpd.socket = ssl.wrap_socket(self.httpd.socket, certfile=self._certfile_path, keyfile=self._keyfile_path, server_side=True)

        
class DialogFlowHandler(s.BaseHTTPRequestHandler):
    """
    The HTTPS response to react the POST from Google DialogFlow and publish a ROS topic.
    """
    def __init__(self, *args):
        self.pub = rospy.Publisher('dialog_response', DialogResponse, queue_size=1)
        s.BaseHTTPRequestHandler.__init__(self, *args)
    
    def do_POST(self):
        print('Got POST request')
        user_agent = self.headers.get("User-Agent")
        rospy.loginfo(self.headers)
        # if user_agent == "Google-Dialogflow":
        self._parse_json()
        self._pub_task()
        self._response()
                
    def _parse_json(self):
        content_len = int(self.headers.get("content-length"))
        request_body = self.rfile.read(content_len).decode("utf-8")
        self.json_content = json.loads(request_body)

    def _pub_task(self):
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
        # print(msg)
        self.pub.publish(msg)
        
    def _response(self):
        # update url param
        url_parsed = urlparse(self.path)
        params = parse_qs(url_parsed.query)
        # make response
        body  = "method: " + str(self.command) + "\n"
        body += "params: " + str(params) + "\n"
        # body += "body  : " + req_body + "\n"
        self.send_response(200)
        self.send_header('Content-type', 'text/html; charset=utf-8')
        self.send_header('Content-length', len(body.encode()))
        self.end_headers()
        self.wfile.write(body.encode())

    def _bad_request(self):
        self.send_response(400)
        self.end_headers()

        
if __name__ == '__main__':
    try:
        # logging.basicConfig(level=logging.DEBUG)
        server = Server()
        rospy.loginfo('DialogFlow HTTPS Server starts - %s:%s' % (server.host, server.port))

    except:
        pass

    while not rospy.is_shutdown():
        server.httpd.serve_forever()

    server.killnode()
