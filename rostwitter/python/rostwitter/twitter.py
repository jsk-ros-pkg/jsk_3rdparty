# originally from https://raw.githubusercontent.com/bear/python-twitter/v1.1/twitter.py  # NOQA

import json as simplejson
import requests
from requests_oauthlib import OAuth1
from StringIO import StringIO

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
        if 'error' in data:
            raise Exception(data)
        return data

    def post_media(self, status, media):
        # 116 = 140 - len("http://t.co/ssssssssss")
        if len(status) > 116:
            rospy.logwarn('tweet wit media is too longer > 116 characters')
            status = status[:116]
        url = 'https://api.twitter.com/1.1/statuses/update_with_media.json'
        data = {'status': StringIO(status)}
        data['media'] = open(str(media), 'rb').read()
        json = self._request_url(url, 'POST', data=data)
        data = simplejson.loads(json.content)
        if 'errors' in data:
            raise Exception(data)
        return data
