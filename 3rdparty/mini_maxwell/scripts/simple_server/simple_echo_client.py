#!/usr/bin/env python

from socket import *
import sys
import argparse
import struct

# use raw_input for python2 c.f. https://stackoverflow.com/questions/5868506/backwards-compatible-input-calls-in-python
if hasattr(__builtins__, 'raw_input'):
    input = raw_input

parser = argparse.ArgumentParser(description='Simple socket client')
parser.add_argument("--port", default=8080, type=int)
parser.add_argument("--udp", action="store_true")
parser.add_argument("--ip", default="127.0.0.1")
args = parser.parse_args()

print("connecting to ", (args.ip, args.port))

if not args.udp:
    server = socket(AF_INET, SOCK_STREAM)
    server.connect((args.ip, args.port))
else:
    server = socket(AF_INET, SOCK_DGRAM)
    while True:
        data = input('> ')
        if not data:
            break
        packer = struct.Struct("!%ds" % len(data))
        data = packer.pack(data)
        print("sending", packer.size * 8, "bits")
        if not args.udp:
            server.send(data)
        else:
            server.sendto(data, (args.ip, args.port))
        if not data:
            break
        print(data)
    server.close()
