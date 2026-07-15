#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from std_msgs.msg import String


class SpeechRecognitionCandidatesToString(Node):
    def __init__(self):
        super().__init__('speech_recognition_candidates_to_string')
        self.publisher = self.create_publisher(String, '~/output', 1)
        self.subscription = self.create_subscription(
            SpeechRecognitionCandidates, '~/input', self._callback, 1)

    def _callback(self, candidates):
        if not candidates.transcript:
            self.get_logger().warning(
                'Received speech recognition candidates without a transcript')
            return
        message = String()
        message.data = candidates.transcript[0]
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = SpeechRecognitionCandidatesToString()
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
