#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sound_play_msgs.action import SoundRequest as SoundRequestAction
from sound_play_msgs.msg import SoundRequest
from speech_recognition_msgs.msg import SpeechRecognitionCandidates


class ParrotNode(Node):
    def __init__(self):
        super().__init__('parrot_node')
        self.declare_parameter('tts_action_name', 'sound_play')
        self.declare_parameter('confidence_threshold', 0.8)
        self.confidence_threshold = self.get_parameter(
            'confidence_threshold').value
        self.sound_client = ActionClient(
            self,
            SoundRequestAction,
            self.get_parameter('tts_action_name').value,
        )
        self.subscription = self.create_subscription(
            SpeechRecognitionCandidates, '~/input', self._callback, 1)
        self._server_warning_logged = False

    def _callback(self, message):
        if not message.transcript:
            return
        if message.confidence and \
                message.confidence[0] < self.confidence_threshold:
            return
        if not self.sound_client.server_is_ready():
            if not self._server_warning_logged:
                self.get_logger().warning(
                    'sound_play action is not ready; transcript is skipped')
                self._server_warning_logged = True
            return

        request = SoundRequest()
        request.sound = SoundRequest.SAY
        request.command = SoundRequest.PLAY_ONCE
        request.volume = 1.0
        request.arg = message.transcript[0]
        goal = SoundRequestAction.Goal()
        goal.sound_request = request
        self.sound_client.send_goal_async(goal)


def main(args=None):
    rclpy.init(args=args)
    node = ParrotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
