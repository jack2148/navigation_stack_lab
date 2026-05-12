#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import time


class CaptureDonePub(Node):
    def __init__(self, place_id):
        super().__init__('capture_done_pub')
        self._place_id = place_id
        self._pub = self.create_publisher(String, '/patrol/capture_done', 10)
        # subscriber 연결 대기 후 1회 발행
        self._timer = self.create_timer(0.5, self._publish_once)

    def _publish_once(self):
        self._timer.cancel()
        msg = String()
        msg.data = f'done:{self._place_id}'
        self._pub.publish(msg)
        self.get_logger().info(f'Published -> /patrol/capture_done : "{msg.data}"')


def main():
    if len(sys.argv) < 2:
        print('Usage: python3 capture_done_pub.py <place_id>')
        print('  ex)  python3 capture_done_pub.py place_001')
        print()
        print('현재 대기 중인 place_id 확인:')
        print('  ros2 topic echo /patrol/capture_trigger')
        sys.exit(1)

    place_id = sys.argv[1]
    rclpy.init()
    node = CaptureDonePub(place_id)
    rclpy.spin_once(node, timeout_sec=2.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
