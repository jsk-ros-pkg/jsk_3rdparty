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

from std_msgs.msg import String
import requests
import json
import sys
import re

try:
    from jsk_ros1_ros2_compat import rospy_rclpy_compat as ros_compat
    from jsk_ros1_ros2_compat.rospy_rclpy_compat import ROS_VERSION
except ImportError:
    # jsk_common < 3.0 has no jsk_ros1_ros2_compat, so this is ROS1 only:
    # forward ros_compat.xxx(node, ...) to rospy.xxx(...)
    import rospy
    ROS_VERSION = 1
    class ros_compat(object):
        create_publisher = staticmethod(lambda node, *args, **kwargs: rospy.Publisher(*args, **kwargs))
        create_subscription = staticmethod(lambda node, *args, **kwargs: rospy.Subscriber(*args, **kwargs))
        get_param = staticmethod(lambda node, *args, **kwargs: rospy.get_param(*args, **kwargs))
        init_node = staticmethod(lambda name, **kwargs: rospy.init_node(name, **kwargs))
        spin = staticmethod(lambda node: rospy.spin())
        loginfo = staticmethod(lambda node, *args, **kwargs: rospy.loginfo(*args, **kwargs))
        logerr = staticmethod(lambda node, *args, **kwargs: rospy.logerr(*args, **kwargs))
        logdebug = staticmethod(lambda node, *args, **kwargs: rospy.logdebug(*args, **kwargs))

if ROS_VERSION == 2:
    from ament_index_python.packages import get_package_share_directory
else:
    import rospy
    import rospkg

if sys.version_info.major == 2:
    reload(sys)
    sys.setdefaultencoding('utf-8')


class ChaplusROS(object):

    def __init__(self, node=None):

        self.node = node
        self.chatbot_engine = ros_compat.get_param(node, "~chatbot_engine", "Mebo")
        self.use_sample = ros_compat.get_param(node, "~use_sample", True)
        # please write your apikey to chaplus_ros/apikey.json
        if ROS_VERSION == 2:
            package_path = get_package_share_directory('chaplus_ros')
        else:
            package_path = rospkg.RosPack().get_path('chaplus_ros')
        apikey_path = ros_compat.get_param(
            node, "~chaplus_apikey_file", package_path+"/apikey.json")
        try:
            with open(apikey_path) as j:
                apikey_json = json.loads(j.read())
        except Exception as e:
            ros_compat.logerr(self.node, 'Could not find {}'.format(apikey_path))
            ros_compat.logerr(self.node, 'please add your API keys to chaplus, for example')
            ros_compat.logerr(self.node,
                "echo '{\"apikey\": \00000000\"}' > `rospack find chaplus_ros`/apikey.json")
            sys.exit(e)

        if self.use_sample==True:
            communication_sample_path=ros_compat.get_param(
                node, "~communication_sample_file", package_path+"/communication_sample.json")
            try:
                with open(communication_sample_path) as j2:
                    self.communication_sample_json = json.loads(j2.read(), strict=False)
            except Exception as e:
                ros_compat.logerr(self.node, 'Could not find {}'.format(communication_sample_path))
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
            ros_compat.logerr(self.node, "please use chatbot_engine Chaplus or A3RT")
            sys.exit(1)

        # define pub/sub
        self.pub = ros_compat.create_publisher(node, 'response', String, queue_size=1)
        ros_compat.create_subscription(node, "request", String, self.topic_cb)

    def topic_cb(self, msg):
        response_json = None
        # use chaplus
        if self.chatbot_engine=="Chaplus":
            try:
                ros_compat.loginfo(self.node, "received {}".format(msg.data))

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
                ros_compat.logerr(self.node, "Failed to reqeust url={}, headers={}, data={}".format(
                    self.url, self.headers, self.data))
                ros_compat.logerr(self.node, str(e))
                best_response = "ごめんなさい、よくわからないです"
            ros_compat.loginfo(self.node, "chaplus: returns best response {}".format(best_response))

        #use A3RT
        elif self.chatbot_engine=="A3RT":
            try:
                ros_compat.loginfo(self.node, "received {}".format(msg.data))
                params = {"apikey": self.apikey, "query": msg.data,}
                response = requests.post(self.endpoint, params)
                response_json = response.json()
                if 'results' not in response_json:
                    best_response = "ごめんなさい、よくわからないです"
                else:
                    best_response = response_json["results"][0]["reply"]
            except Exception as e:
                ros_compat.logerr(self.node, "Failed to reqeust url={}, data={}".format(
                    self.endpoint, msg.data))
                ros_compat.logerr(self.node, str(e))
                best_response = "ごめんなさい、よくわからないです"
            ros_compat.loginfo(self.node, "a3rt: returns best response {}".format(best_response))

        #use Mebo
        elif self.chatbot_engine == "Mebo":
            try:
                ros_compat.loginfo(self.node, "received {}".format(msg.data))
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
                ros_compat.logerr(self.node, "Failed to reqeust url={}, headers={}, data={}".format(
                    self.url, self.headers, self.data))
                ros_compat.logerr(self.node, str(e))
                best_response = "ごめんなさい、よくわからないです"
            ros_compat.loginfo(self.node, "mebo: returns best response {}".format(best_response))
        else:
            ros_compat.logerr(self.node, "please use chatbot_engine Chaplus or A3RT or Mebo")

        if response_json is not None:
            # convert to string for print out
            if sys.version_info.major == 2:
                ros_compat.logdebug(self.node, str(json.dumps(response_json, indent=2,
                                              ensure_ascii=False, encoding='unicode-escape')))
            else:  # pytyon3
                ros_compat.logdebug(self.node, json.dumps(
                    response_json, indent=2, ensure_ascii=False))

        #publish response
        self.pub.publish(String(data=best_response))

if __name__ == '__main__':
    node = ros_compat.init_node('chaplus_ros')
    ChaplusROS(node)
    ros_compat.spin(node)
