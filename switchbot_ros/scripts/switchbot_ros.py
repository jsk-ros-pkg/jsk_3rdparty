#!/usr/bin/env python
import time

import rospy
from switchbot_ros.srv import Command
from switchbot_ros.srv import CommandResponse
from switchbot_ros.msg import CommandTopic
from switchbot import SwitchBotRequest

class SwitchBotROS:
    """
    Control your switchbot with IFTTT.
    Please setup your SwitchBot device as the README shows.
    """
    def __init__(self):
        rospy.init_node('switchbot_server')
        self._ifttt_key = rospy.get_param('~ifttt_key')
        self.on_server = rospy.Service('~on', Command, self.on)
        self.off_server = rospy.Service('~off', Command, self.off)
        self.press_server = rospy.Service('~press', Command, self.press)
        self.sub = rospy.Subscriber('~command', CommandTopic, self.callback)

    def callback(self, data):
        if data.command == 'on':
            self.on(data)
        elif data.command == 'off':
            self.off(data)
        elif data.command == 'press':
            if data.times > 1:
                for t in range(data.times):
                    self.press(data)
                    time.sleep(data.sleep)
            else:
                self.press(data)
        else:
            rospy.logerr('Unknown command was subscribed.')
        
    def _post_command(self, data, command):
        # define response
        resp = CommandResponse()
        # send GET method
        event = data.nickname[1:].replace('/', '_') + '_' + command
        key = self._ifttt_key
        switchbot_request = SwitchBotRequest(event=event, key=key)
        switchbot_request.request()
        # check HTTP status
        resp.status = switchbot_request.status
        resp.msg = switchbot_request.msg
        if resp.status == 200:
            resp.successful = True
            rospy.loginfo('Successfully send the GET to IFTTT server. status:{}, msg:{}'.format(switchbot_request.status, switchbot_request.msg))
        else:
            resp.successful = False
            rospy.logerr('IFTTT HTTP error! status:{}, msg:{}'.format(switchbot_request.status, switchbot_request.msg))
        return resp

    def on(self, data):
        return self._post_command(data, 'on')

    def off(self, data):
        return self._post_command(data, 'off')

    def press(self, data):
        return self._post_command(data, 'press')

if __name__ == '__main__':
    try:
        node = SwitchBotROS()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
