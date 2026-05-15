#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PointStamped


class TrackingNode(Node):
    def __init__(self):
        super().__init__('tracking_node')

        self.safe_distance = 1.3   # 사람까지 유지할 목표 거리 (m)
        self.arrived_threshold = 0.4  # safe_distance 내 진입 판정 오차 (m)

        self.tracking_state = 'IDLE'
        self.auth_published = False  # 도착 pub 중복 방지

        self.follow_state_sub = self.create_subscription(
            String,
            '/person_tracking/follow_state',
            self.follow_state_callback,
            10
        )
        self.follow_target_sub = self.create_subscription(
            PointStamped,
            '/person_tracking/follow_target',
            self.target_callback,
            10
        )

        self.cmd_vel_pub = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        self.auth_pub = self.create_publisher(Bool, '/auth_ready', 10)

        self.get_logger().info('Tracking node started.')

    def follow_state_callback(self, msg):
        new_state = msg.data.strip().upper()

        if new_state == self.tracking_state:
            return

        self.tracking_state = new_state
        self.get_logger().info(f'[STATE] {new_state}')

        if new_state != 'TRACKING':
            self.stop_robot()
            self.auth_published = False

    def target_callback(self, msg):
        if self.tracking_state != 'TRACKING':
            return

        # RealSense optical frame: z=전방 거리, x=좌우(오른쪽+)
        forward = msg.point.z
        lateral = -msg.point.x  # 카메라 오른쪽+ → 로봇 왼쪽+

        error_d = forward - self.safe_distance

        # 목표 거리 도달 판정
        if abs(error_d) <= self.arrived_threshold:
            self.stop_robot()
            if not self.auth_published:
                auth_msg = Bool()
                auth_msg.data = True
                self.auth_pub.publish(auth_msg)
                self.auth_published = True
                self.get_logger().info('[ARRIVED] /auth_ready published.')
            return

        twist = Twist()

        # 조향 제어 (Angular z) - 최대 ±0.8 rad/s
        error_yaw = math.atan2(lateral, forward)
        twist.angular.z = max(-0.8, min(0.8, 1.2 * error_yaw))

        # 전진 제어 (Linear x) - 최대 0.4 m/s
        if error_d > 0.0:
            twist.linear.x = max(0.0, min(0.6, 0.6 * error_d))

        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = TrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
