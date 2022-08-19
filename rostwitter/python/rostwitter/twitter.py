# originally from https://raw.githubusercontent.com/bear/python-twitter/v1.1/twitter.py  # NOQA

import json as simplejson
import requests
from itertools import zip_longest
from requests_oauthlib import OAuth1

import rospy

from rostwitter.util import count_tweet_text
from rostwitter.util import split_tweet_text
from rostwitter.cv_util import extract_media_from_text


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

    def _post_update_with_reply(self, texts, media_list=None,
                                in_reply_to_status_id=None):
        split_media_list = []
        media_list = media_list or []
        for i in range(0, len(media_list), 4):
            split_media_list.append(media_list[i * 4:(i + 1) * 4])
        for text, media_list in zip_longest(texts, split_media_list):
            text = text or ''
            media_list = media_list or []
            url = 'https://api.twitter.com/1.1/statuses/update.json'
            data = {'status': text}
            media_ids = self._upload_media(media_list)
            if len(media_ids) > 0:
                data['media_ids'] = media_ids
            if in_reply_to_status_id is not None:
                data['in_reply_to_status_id'] = in_reply_to_status_id
            json = self._request_url(url, 'POST', data=data)
            data = simplejson.loads(json.content)
            in_reply_to_status_id = data['id']
        return data

    def _upload_media(self, media_list):
        url = 'https://upload.twitter.com/1.1/media/upload.json'
        media_ids = []
        for media in media_list:
            data = {'media': media}
            r = self._request_url(url, 'POST', data=data)
            if r.status_code == 200:
                rospy.loginfo('upload media success')
                media_ids.append(str(r.json()['media_id']))
            else:
                rospy.logwarn('upload media failed')
        media_ids = ','.join(media_ids)
        return media_ids

    def post_update(self, status):
        media_list, status = extract_media_from_text(status)
        media_ids = self._upload_media(media_list[:4])
        texts = split_tweet_text(status)
        status = texts[0]
        url = 'https://api.twitter.com/1.1/statuses/update.json'
        data = {'status': status}
        if len(media_ids) > 0:
            data['media_ids'] = media_ids
        json = self._request_url(url, 'POST', data=data)
        data = simplejson.loads(json.content)
        if len(texts) > 1 or len(media_list) > 4:
            data = self._post_update_with_reply(
                texts[1:],
                media_list=media_list[4:],
                in_reply_to_status_id=data['id'])
        return data

    def post_media(self, status, media, in_reply_to_status_id=None):
        texts = split_tweet_text(status)
        status = texts[0]
        url = 'https://api.twitter.com/1.1/statuses/update_with_media.json'
        data = {'status': status}
        data['media'] = open(str(media), 'rb').read()
        json = self._request_url(url, 'POST', data=data)
        data = simplejson.loads(json.content)
        if len(texts) > 1:
            data = self._post_update_with_reply(
                texts[1:],
                in_reply_to_status_id=data['id'])
        return data
