#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import rospy
import Queue

from dialogflow_task_executive.msg import ConversationText

def print_message(msg):

    print('type: {}, id: {}, text: {}'.format(
                msg.conversation_type,
                msg.conversation_id,
                msg.text
                ))

class TextChatPrompt:

    def __init__(self):

        self.conversation_type = rospy.get_param('~conversation_type', 'text')
        self.conversation_id = rospy.get_param('~conversation_id', 1)
        self.duration_response = rospy.get_param('~duration_response', 10.0)
        self.queue_text = Queue.Queue()
        self.pub_text = rospy.Publisher('dialogflow_text_input', ConversationText, queue_size=1)
        self.sub_text = rospy.Subscriber('dialogflow_text_reply', ConversationText, self.callback)

    def callback(self, msg):

        if msg.conversation_id == self.conversation_id:
            self.queue_text.put(msg)

    def spin(self):

        while not rospy.is_shutdown():

            text = raw_input('Input text > ')

            msg = ConversationText(
                    conversation_type=self.conversation_type,
                    conversation_id=self.conversation_id,
                    text=text)
            self.pub_text.publish(msg)

            rate = rospy.Rate(10)
            timeout = rospy.Time.now() + rospy.Duration(self.duration_response)
            while not rospy.is_shutdown():
                rate.sleep()
                if rospy.Time.now() > timeout:
                    rospy.logerr('Timeout')
                    break
                if not self.queue_text.empty():
                    msg = self.queue_text.get()
                    print_message(msg)
                    break


def main():
    rospy.init_node('text_chat_prompt')
    node = TextChatPrompt()
    node.spin()


if __name__ == '__main__':
    main()
