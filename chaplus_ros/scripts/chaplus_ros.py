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

if sys.version_info.major == 2:
    reload(sys)
    sys.setdefaultencoding('utf-8')


class ChaplusROS(object):

    def __init__(self):

        # APIKEYの部分は自分のAPI鍵を代入してください
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

        self.headers = {'content-type': 'text/json'}
        self.url = 'https://www.chaplus.jp/v1/chat?apikey={}'.format(
            apikey_json['apikey'])

        # define pub/sub
        self.pub = rospy.Publisher('response', String, queue_size=1)
        rospy.Subscriber("request", String, self.topic_cb)

    def topic_cb(self, msg):
        # chaplusを利用
        try:
            rospy.loginfo("received {}".format(msg.data))
            self.data = json.dumps({'utterance': msg.data})
            response = requests.post(
                url=self.url, headers=self.headers, data=self.data)
            response_json = response.json()
            if not response_json.has_key('bestResponse'):
                raise Exception(response_json)
        except Exception as e:
            rospy.logerr("Failed to reqeust url={}, headers={}, data={}".format(
                self.url, self.headers, self.data))
            rospy.logerr(e)
            return None

        # convert to string for print out
        if sys.version_info.major == 2:
            rospy.logdebug(str(json.dumps(response_json, indent=2,
                                          ensure_ascii=False, encoding='unicode-escape')))
        else:  # pytyon3
            rospy.logdebug(json.dumps(
                response_json, indent=2, ensure_ascii=False))

        # publish best response
        best_response = response_json['bestResponse']['utterance']
        rospy.loginfo("returns best response {}".format(best_response))
        self.pub.publish(String(best_response))


if __name__ == '__main__':
    rospy.init_node('chaplus_ros')
    ChaplusROS()
    rospy.spin()
