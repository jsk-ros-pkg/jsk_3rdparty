#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


import sys
import socket
import struct
import rospy
from threading import Thread, Event

class SocketTransport(Thread):
    def __init__(self, host, port, max_retry):
        super(SocketTransport, self).__init__()
        self.host = host
        self.port = port
        self.max_retry = max_retry  # if 0, try connect forever
        self.send_buffer_size = 1024
        self.recv_buffer_size = 1024

        self.socket = None

        # var for thread
        self.poll_rate = 0.1  # 10 Hz
        self.alive = None
        self.callbacks = []

    def start(self):
        self.alive = Event()
        self.connect()
        super(SocketTransport, self).start()
        rospy.loginfo("[%s] Started" % type(self).__name__)

    def join(self):
        if self.alive is not None:
            self.alive.set()
        if self.is_connected():
            self.disconnect()
        if self.is_alive():
            super(SocketTransport, self).join()
        rospy.loginfo("[%s] Stopped" % type(self).__name__)

    def run(self):
        recvbuf = ""
        while not self.alive.wait(self.poll_rate):
            try:
                self.socket.settimeout(1)
                buf = self.socket.recv(self.recv_buffer_size)
                if sys.version_info.major >= 3 and type(buf) == bytes: # on Python3 convert buf to str
                    buf = buf.decode()
                recvbuf += buf
            except socket.error as e:
                if sys.version_info.major >= 3 and type(e) == socket.timeout:
                    continue
                if sys.version_info.major <  3 and e.message == "timed out":
                    continue
                self.reconnect()
            try:
                parsed_data, parsed_length = self.parse(recvbuf)
                for data in parsed_data:
                    for cb in self.callbacks:
                        cb(data)
                if len(recvbuf) > parsed_length:
                    recvbuf = recvbuf[parsed_length:]
                else:
                    recvbuf = ""
            except ValueError:
                pass
            except RuntimeError as e:
                rospy.logerr("Failed to parse data: %s" % str(e))

    def is_connected(self):
        if self.socket is None:
            return False
        try:
            self.socket.getpeername()
            return True
        except socket.error as e:
            if e.errno == 107:
                # no connection error
                return False
            else:
                raise e

    def connect(self):
        retry = 0
        err = None
        info = (self.host, self.port)
        while not rospy.is_shutdown() and (self.max_retry == 0 or retry < self.max_retry):
            s = None
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, self.send_buffer_size)
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
                s.connect(info)
                rospy.loginfo("[%s] Connected to server %s", type(self).__name__, info)
                self.socket = s
                return True
            except socket.error as e:
                err = e
                if s:
                    s.close()
                retry += 1
                rospy.sleep(1.0)
        raise IOError("[%s] Failed to connect to %s after %d retries: %s" % (type(self).__name__, info, retry, str(err)))

    def disconnect(self):
        self.socket.close()
        self.socket = None
        rospy.loginfo("[%s] Disconnected", type(self).__name__)

    def reconnect(self):
        if self.is_connected():
            self.disconnect()
        self.connect()

    def send(self, data):
        if sys.version_info.major >= 3 and type(data) == str: # on Python3 convert data to bytes
            data = data.encode()
        try:
            self.socket.sendall(data)
        except socket.error:
            self.reconnect()
        except AttributeError:
            self.connect()

    def on_received_data(self, cb):
        self.callbacks.append(cb)

    def parse(self, data):
        raise NotImplementedError()


if __name__ == '__main__':
    pass
