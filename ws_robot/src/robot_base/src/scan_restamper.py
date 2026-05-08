#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanRestamper(Node):
    def __init__(self):
        super().__init__('scan_restamper')
        self.sub = self.create_subscription(LaserScan, '/scan_pre', self.cb, 10)
        self.pub = self.create_publisher(LaserScan, '/scan', 10)

    def cb(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanRestamper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
