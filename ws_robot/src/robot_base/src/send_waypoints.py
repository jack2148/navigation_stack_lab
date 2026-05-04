#!/usr/bin/env python3
"""
웨이포인트 2개를 patrol_node에 직접 전송하는 스크립트.
GUI 없이 터미널에서 순찰을 시작할 때 사용합니다.

사용법:
  python3 send_waypoints.py

좌표 수정:
  아래 WAYPOINTS 리스트의 x, y, yaw 값을 원하는 위치로 변경하세요.
  yaw 단위: 라디안 (예: 0.0=정면, 1.57=좌, 3.14=뒤, -1.57=우)
"""
import time
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# ---------------------------------------------------------------
# 웨이포인트 설정 (이 부분만 수정하면 됩니다)
# ---------------------------------------------------------------
WAYPOINTS = [
    {"place_id": "WP_1", "x": 1.0, "y": 0.0, "yaw": 0.0,  "patrol_order": 1},
    {"place_id": "WP_2", "x": 2.0, "y": 0.0, "yaw": 3.14, "patrol_order": 2},
]
# ---------------------------------------------------------------


def main():
    rclpy.init()
    node = Node('send_waypoints')

    pub = node.create_publisher(String, '/patrol/waypoints_json', 10)

    # 구독자가 연결될 때까지 잠시 대기
    time.sleep(1.0)

    msg = String()
    msg.data = json.dumps({"places": WAYPOINTS})
    pub.publish(msg)

    node.get_logger().info(f'웨이포인트 {len(WAYPOINTS)}개 전송 완료:')
    for wp in WAYPOINTS:
        node.get_logger().info(
            f'  [{wp["patrol_order"]}] {wp["place_id"]} '
            f'x={wp["x"]}, y={wp["y"]}, yaw={wp["yaw"]}'
        )

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
