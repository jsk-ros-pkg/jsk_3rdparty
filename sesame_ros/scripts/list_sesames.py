#!/usr/bin/env python

from __future__ import print_function

import argparse
import os.path as osp
import pprint
import sys

import requests


def list_sesames(auth_token):
    if osp.isfile(osp.expanduser(auth_token)):
        with open(osp.expanduser(auth_token), 'r') as f:
            auth_token = f.readline().rstrip()

    ret = requests.get(
        'https://api.candyhouse.co/public/sesames',
        headers={'Authorization': auth_token})
    if ret.status_code == 200:
        pprint.pprint(ret.json())
    else:
        print('[HTTP status code: {}]\n'
              '{}\n'
              'No Sesames found.'.format(
                  ret.status_code, ret.text), file=sys.stderr)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'auth_token', type=str,
        help='Your authorization token or path to its file.')
    args = parser.parse_args()

    list_sesames(args.auth_token)
