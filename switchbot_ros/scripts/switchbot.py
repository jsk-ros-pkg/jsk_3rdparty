#!/usr/bin/env python
import urllib.request
from urllib.error import URLError, HTTPError
import time

class SwitchBotRequest:
    """
    For pressing switchbot with IFTTT.
    Please setup your SwitchBot device as the README shows.
    """
    def __init__(self, event, key):
        self.event = event
        self.key = key
        self.url = "https://maker.ifttt.com/trigger/" + self.event + "/with/key/" + self.key
        self.status = None

    def request(self):
        req = urllib.request.Request(self.url)
        try:
            with urllib.request.urlopen(req) as res:
                self._body = res
                self.status = self._body.status
                self.msg = self._body.msg
        except HTTPError as e:
            self._body = e
            self.status = self._body.status
            self.msg = self._body.msg
            print('Got HTTPError. status:{} msg:{}'.format(e.code, e.msg))
        except URLError as e:
            self._body = e
            self.status = 0
            self.msg = self._body.reason
            print('Got URLError. reason:{}'.format(e.reason))
        
    def continuous_request(self, times, margin_sec):
       # add the least margin_sec
       for t in range(times):
           self.request()
           time.sleep(margin_sec)
