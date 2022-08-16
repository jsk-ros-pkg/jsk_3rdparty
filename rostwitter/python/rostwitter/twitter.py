# originally from https://raw.githubusercontent.com/bear/python-twitter/v1.1/twitter.py  # NOQA

import json as simplejson
import requests
from requests_oauthlib import OAuth1
# https://stackoverflow.com/questions/11914472/stringio-in-python3
try:
    from StringIO import StringIO ## for Python 2
except ImportError:
    from io import StringIO ## for Python 3

import rospy

from rostwitter.util import count_tweet_text
from rostwitter.util import split_tweet_text


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

    def _check_and_split_word(self, text):
        """Check tweet text length and split it.

        See https://developer.twitter.com/en/docs/counting-characters

        """
        c = count_tweet_text(text)
        if c > 280:
            rospy.logwarn('tweet is too longer > 280 characters.')
            texts = split_tweet_text(text)
            if len(texts) > 0:
                text = texts[0]
            else:
                text = ''
        return text

    def post_update(self, status):
        status = self._check_and_split_word(status)
        url = 'https://api.twitter.com/1.1/statuses/update.json'
        data = {'status': StringIO(status)}
        json = self._request_url(url, 'POST', data=data)
        data = simplejson.loads(json.content)
        return data

    def post_media(self, status, media):
        status = self._check_and_split_word(status)
        url = 'https://api.twitter.com/1.1/statuses/update_with_media.json'
        data = {'status': StringIO(status)}
        data['media'] = open(str(media), 'rb').read()
        json = self._request_url(url, 'POST', data=data)
        data = simplejson.loads(json.content)
        return data
