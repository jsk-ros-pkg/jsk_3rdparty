#!/usr/bin/env python                                                                                              
import imp  ## for rosbuild                                                                                       
import rospy
import yaml,sys
import re, os
from io import BytesIO
from StringIO import StringIO

from std_msgs.msg import String

global Api, CKEY, CSECRET, AKEY, ASECRET

import requests
from requests_oauthlib import OAuth1
import base64
import json as simplejson

# https://raw.githubusercontent.com/bear/python-twitter/v1.1/twitter.py

class twitter(object):
    def __init__(self,
                 consumer_key=None,
                 consumer_secret=None,
                 access_token_key=None,
                 access_token_secret=None):
        self._consumer_key        = consumer_key
        self._consumer_secret     = consumer_secret
        self._access_token_key    = access_token_key
        self._access_token_secret = access_token_secret

        self.__auth = OAuth1(self._consumer_key, self._consumer_secret,
                             self._access_token_key, self._access_token_secret)
        self._requests_timeout = 60

    def _RequestUrl(self, url, verb, data=None):
        if verb == 'POST':
            print(data)
            if data.has_key('media'):
                return requests.post(
                    url,
                    files=data,
                    auth=self.__auth,
                    timeout=self._requests_timeout
                )
            else:
                return requests.post(
                    url,
                    data=data,
                    auth=self.__auth,
                    timeout=self._requests_timeout
                )
        if verb == 'GET':
            url = self._BuildUrl(url, extra_params=data)
            return requests.get(
                url,
                auth=self.__auth,
                timeout=self._requests_timeout
                )
        return 0  # if not a POST or GET request

    def PostUpdate(self, status):
        url = 'https://api.twitter.com/1.1/statuses/update.json'

        data = {'status': StringIO(status)}
        json = self._RequestUrl(url, 'POST', data=data)
        data = simplejson.loads(json.content)
        if 'error' in data:
            raise Exception(data)
        return data

    def PostMedia(self, status, media):
        url = 'https://api.twitter.com/1.1/statuses/update_with_media.json'

        data = {'status': StringIO(status)}
        data['media'] = open(str(media), 'rb').read()
        json = self._RequestUrl(url, 'POST', data=data)
        data = simplejson.loads(json.content)
        if 'errors' in data:
            raise Exception(data)
        return data

def tweet(dat):
    global Api
    message = dat.data
    rospy.loginfo(rospy.get_name() + " sending %s", message)

    # search word start from / and end with {.jpeg,.jpg,.png,.gif}
    m = re.search('/\S+\.(jpeg|jpg|png|gif)', message)
    ret = None
    if m:
        filename = m.group(0)
        message = re.sub(filename,"",message)
        if os.path.exists(filename):
            rospy.loginfo(rospy.get_name() + " tweet %s with file %s", message, filename)
            ret = Api.PostMedia(message[0:116], filename) # 140 - len("http://t.co/ssssssssss")
            #ret = Api.PostUpdate(message)
        else:
            rospy.logerr(rospy.get_name() + " %s could not find", filename)
    else:
        ret = Api.PostUpdate(message[0:140])
    ## seg faults if message is longer than 140 byte ???                           
        
    rospy.loginfo(rospy.get_name() + " receiving %s", ret)                                                      
    return

def load_oauth_settings():
    global CKEY, CSECRET, AKEY, ASECRET
    account_info = rospy.get_param('account_info', '/var/lib/robot/account.yaml')

    try:
        key = yaml.load(open(account_info))
        CKEY = key['CKEY']
        CSECRET = key['CSECRET']
        AKEY = key['AKEY']
        ASECRET = key['ASECRET']
    except IOError as e:
        rospy.logerr('"%s" not found'%account_info)
        rospy.logerr("$ get access token from https://apps.twitter.com/")
        rospy.logerr("cat /var/lib/robot/%s <<EOF"%account_info)
        rospy.logerr("CKEY: xxx")
        rospy.logerr("CSECRET: xxx")
        rospy.logerr("AKEY: xxx")
        rospy.logerr("ASECRET: xxx")
        rospy.logerr("EOF")
        sys.exit(-1)

if __name__ == '__main__':
    global Api
    rospy.init_node('rostwitter', anonymous=True)
    load_oauth_settings()
    Api = twitter(consumer_key=CKEY,
                  consumer_secret=CSECRET,
                  access_token_key=AKEY,
                  access_token_secret=ASECRET)
    rospy.Subscriber("tweet", String, tweet)
    rospy.spin()




