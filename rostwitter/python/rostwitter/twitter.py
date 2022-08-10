# originally from https://raw.githubusercontent.com/bear/python-twitter/v1.1/twitter.py  # NOQA

import base64
import json as simplejson
import requests
from requests_oauthlib import OAuth1
# https://stackoverflow.com/questions/11914472/stringio-in-python3
try:
    from StringIO import StringIO ## for Python 2
except ImportError:
    from io import StringIO ## for Python 3

import os
import rospy


class Twitter(object):
    def __init__(
            self,
            consumer_key=None,
            consumer_secret=None,
            access_token_key=None,
            access_token_secret=None
    ):
        self._consumer_key = consumer_key
        self._consumer_secret = consumer_secret
        self._access_token_key = access_token_key
        self._access_token_secret = access_token_secret

        self.__auth = OAuth1(self._consumer_key, self._consumer_secret,
                             self._access_token_key, self._access_token_secret)
        self._requests_timeout = 60

    def _request_url(self, url, verb, data=None):
        if verb == 'POST':
            if 'media' in data:
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

    def post_update(self, status):
        if len(status) > 140:
            rospy.logwarn('tweet is too longer > 140 characters')
            status = status[:140]
        url = 'https://api.twitter.com/1.1/statuses/update.json'
        data = {'status': StringIO(status)}
        json = self._request_url(url, 'POST', data=data)
        data = simplejson.loads(json.content)
        return data

    def post_media(self, status, media):
        # 116 = 140 - len("http://t.co/ssssssssss")
        if len(status) > 116:
            rospy.logwarn('tweet with media is too longer > 116 characters')
            status = status[:116]
        url = 'https://api.twitter.com/1.1/statuses/update_with_media.json'
        data = {'status': StringIO(status)}
        if os.path.exists(str(media)):
            data['media'] = open(str(media), 'rb').read()
        else:
            try:
                if base64.b64encode(base64.b64decode(media)) == media:
                    data['media'] = base64.b64decode(media)
                else:
                    raise Exception
            except:
                rospy.logwarn('tweet media is neither file nor base64 data')
        json = self._request_url(url, 'POST', data=data)
        data = simplejson.loads(json.content)
        return data
