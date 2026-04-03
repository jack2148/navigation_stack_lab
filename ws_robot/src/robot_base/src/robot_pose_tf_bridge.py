#!/usr/bin/env python3
"""Publish robot pose as geometry_msgs/Pose2D for GUI integration.

Default behavior:
- Lookup TF from map -> base_link
- Publish geometry_msgs/Pose2D on /robot_pose

Why this matches your GUI contract:
- x, y, theta are sent in robot/world control coordinates (meters, radians)
- GUI should convert to pixel coordinates using map.yaml resolution/origin

Usage examples:
  python3 robot_pose_tf_bridge.py
  python3 robot_pose_tf_bridge.py --ros-args -p parent_frame:=odom
  python3 robot_pose_tf_bridge.py --ros-args -p output_topic:=/robot_pose
"""

import math
from typing import Optional

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Pose2D
from tf2_ros import Buffer, TransformException, TransformListener


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Convert quaternion to yaw [rad]."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class RobotPoseTFBridge(Node):
    def __init__(self) -> None:
        super().__init__('robot_pose_tf_bridge')

        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('child_frame', 'base_link')
        self.declare_parameter('output_topic', '/robot_pose')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('transform_timeout_sec', 0.2)

        self.parent_frame: str = self.get_parameter('parent_frame').get_parameter_value().string_value
        self.child_frame: str = self.get_parameter('child_frame').get_parameter_value().string_value
        self.output_topic: str = self.get_parameter('output_topic').get_parameter_value().string_value
        self.publish_rate: float = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.transform_timeout_sec: float = self.get_parameter('transform_timeout_sec').get_parameter_value().double_value

        if self.publish_rate <= 0.0:
            self.get_logger().warn('publish_rate must be > 0. Falling back to 10.0 Hz.')
            self.publish_rate = 10.0

        self.publisher = self.create_publisher(Pose2D, self.output_topic, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._last_warn_ns: Optional[int] = None
        self.timer = self.create_timer(1.0 / self.publish_rate, self.on_timer)

        self.get_logger().info(
            f'Publishing TF pose {self.parent_frame} -> {self.child_frame} to {self.output_topic} as Pose2D'
        )

    def on_timer(self) -> None:
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.parent_frame,
                self.child_frame,
                Time() # Time()은 '가장 최신 시간의 데이터'를 의미합니다.
            )
        except TransformException as ex:
            now_ns = self.get_clock().now().nanoseconds
            if self._last_warn_ns is None or (now_ns - self._last_warn_ns) > 2_000_000_000:
                self.get_logger().warn(
                    f'Cannot lookup transform {self.parent_frame} -> {self.child_frame}: {ex}'
                )
                self._last_warn_ns = now_ns
            return

        pose = Pose2D()
        pose.x = tf_msg.transform.translation.x
        pose.y = tf_msg.transform.translation.y
        q = tf_msg.transform.rotation
        pose.theta = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.publisher.publish(pose)


def main() -> None:
    rclpy.init()
    node = RobotPoseTFBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
