#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
# Copyright (c) 2020, JSK Robotics Lab.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of JSK Robotics, Lab. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Ayaka Fujii <a-fujii@jsk.imi.i.u-tokyo.ac.jp>

import rospy
import rospkg
from std_msgs.msg import String
import requests
import json
import sys
import re

if sys.version_info.major == 2:
    reload(sys)
    sys.setdefaultencoding('utf-8')


class ChaplusROS(object):

    def __init__(self):

        self.chatbot_engine = rospy.get_param("~chatbot_engine", "Mebo")
        self.use_sample = rospy.get_param("~use_sample", True)
        # please write your apikey to chaplus_ros/apikey.json
        r = rospkg.RosPack()
        apikey_path = rospy.get_param(
            "~chaplus_apikey_file", r.get_path('chaplus_ros')+"/apikey.json")
        try:
            with open(apikey_path) as j:
                apikey_json = json.loads(j.read())
        except Exception as e:
            rospy.logerr('Could not find {}'.format(apikey_path))
            rospy.logerr('please add your API keys to chaplus, for example')
            rospy.logerr(
                "echo '{\"apikey\": \00000000\"}' > `rospack find chaplus_ros`/apikey.json")
            sys.exit(e)

        if self.use_sample==True:
            communication_sample_path=rospy.get_param(
                "~communication_sample_file", r.get_path('chaplus_ros')+"/communication_sample.json")
            try:
                with open(communication_sample_path) as j2:
                    self.communication_sample_json = json.loads(j2.read(), strict=False)
            except Exception as e:
                rospy.logerr('Could not find {}'.format(communication_sample_path))
                sys.exit(e)

        if self.chatbot_engine=="Chaplus":
            self.headers = {'content-type': 'text/json'}
            self.url = 'https://www.chaplus.jp/v1/chat?apikey={}'.format(
                apikey_json['apikey'])

        elif self.chatbot_engine=="A3RT":
            self.apikey = apikey_json['apikey_a3rt']
            self.endpoint = "https://api.a3rt.recruit.co.jp/talk/v1/smalltalk"

        elif self.chatbot_engine=="Mebo":
            self.headers = {'content-type': 'application/json'}
            self.url = "https://api-mebo.dev/api"
            self.apikey = apikey_json['apikey_mebo']
            self.agentid = apikey_json['agentid_mebo']
            self.uid = apikey_json['uid_mebo']

        else:
            rospy.logerr("please use chatbot_engine Chaplus or A3RT")
            sys.exit(1)

        # define pub/sub
        self.pub = rospy.Publisher('response', String, queue_size=1)
        rospy.Subscriber("request", String, self.topic_cb)

    def topic_cb(self, msg):
        # use chaplus
        if self.chatbot_engine=="Chaplus":
            try:
                rospy.loginfo("received {}".format(msg.data))

                if self.use_sample==True:
                    self.data =json.dumps(
                       {'utterance': msg.data,
                        'agentstate': self.communication_sample_json["agentState"],
                        'addition': self.communication_sample_json["addition"]
                       })
                else:
                    self.data = json.dumps({'utterance': msg.data})
                response = requests.post(
                    url=self.url, headers=self.headers, data=self.data)
                response_json = response.json()
                if 'bestResponse' not in response_json:
                    best_response = "ごめんなさい、よくわからないです"
                else:
                    best_response = response_json['bestResponse']['utterance']
            except Exception as e:
                rospy.logerr("Failed to reqeust url={}, headers={}, data={}".format(
                    self.url, self.headers, self.data))
                rospy.logerr(e)
                best_response = "ごめんなさい、よくわからないです"
            rospy.loginfo("chaplus: returns best response {}".format(best_response))

        #use A3RT
        elif self.chatbot_engine=="A3RT":
            try:
                rospy.loginfo("received {}".format(msg.data))
                params = {"apikey": self.apikey, "query": msg.data,}
                response = requests.post(self.endpoint, params)
                response_json = response.json()
                if 'results' not in response_json:
                    best_response = "ごめんなさい、よくわからないです"
                else:
                    best_response = response_json["results"][0]["reply"]
            except Exception as e:
                rospy.logerr("Failed to reqeust url={}, data={}".format(
                    self.endpoint, msg.data))
                rospy.logerr(e)
                best_response = "ごめんなさい、よくわからないです"
            rospy.loginfo("a3rt: returns best response {}".format(best_response))

        #use Mebo
        elif self.chatbot_engine == "Mebo":
            try:
                rospy.loginfo("received {}".format(msg.data))
                self.data = json.dumps(
                    {'api_key': self.apikey,
                     'agent_id': self.agentid,
                     'utterance': msg.data,
                     'uid': self.uid
                    })
                response = requests.post(self.url, headers=self.headers, data=self.data, timeout=(3.0, 7.5))
                response_json = response.json()
                if 'bestResponse' not in response_json:
                    best_response = "ごめんなさい、よくわからないです"
                else:
                    best_response = response_json['bestResponse']['utterance']
            except Exception as e:
                rospy.logerr("Failed to reqeust url={}, headers={}, data={}".format(
                    self.url, self.headers, self.data))
                rospy.logerr(e)
                best_response = "ごめんなさい、よくわからないです"
            rospy.loginfo("mebo: returns best response {}".format(best_response))
        else:
            rospy.logerr("please use chatbot_engine Chaplus or A3RT or Mebo")

        if response_json is not None:
            # convert to string for print out
            if sys.version_info.major == 2:
                rospy.logdebug(str(json.dumps(response_json, indent=2,
                                              ensure_ascii=False, encoding='unicode-escape')))
            else:  # pytyon3
                rospy.logdebug(json.dumps(
                    response_json, indent=2, ensure_ascii=False))

        #publish response
        self.pub.publish(String(best_response))

if __name__ == '__main__':
    rospy.init_node('chaplus_ros')
    ChaplusROS()
    rospy.spin()
