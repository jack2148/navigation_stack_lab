#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')
        
        # ---------- [추가] 로봇이 켜질 때 고정된 시작 기준 좌표 (초기 위치) ----------
        # 매번 같은 구석이나 침대 밑 등에서 켜진다면, 그 위치의 맵 좌표를 적어주면 됩니다.
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.initial_yaw = 0.0

        # ---------- 픽셀 변환 없이 실제 맵의 X, Y, Yaw(도 단위) 좌표값 하드코딩 ----------
        # 형식: (맵 상의 x 목표(m), 맵 상의 y 목표(m), 바라볼 각도(degree))
        self.waypoints_world = [
            (-20,  -22.4,  0.0),   # 1번 웨이포인트 (도착 후 북쪽 보기)
            (-17.7,  -22.4, 0.0),    # 2번 웨이포인트 (도착 후 동쪽 보기)
            (-20, -22.4,  90.0)   # 3번 웨이포인트 (도착 후 서쪽 보기)
        ]
        
        self.navigator = BasicNavigator()

    def yaw_to_quaternion(self, yaw_degree):
        # 2D 평면이므로 Roll, Pitch는 고려하지 않고 오일러 각도(Yaw, degree)를 쿼터니언(z, w)으로 변환
        yaw_rad = math.radians(yaw_degree)
        qz = math.sin(yaw_rad / 2.0)
        qw = math.cos(yaw_rad / 2.0)
        return qz, qw

    def generate_poses(self):
        """실제 좌표 리스트를 바로 Nav2 액션용 PoseStamped 배열로 포장"""
        poses = []
        for (x, y, desired_yaw) in self.waypoints_world:
            qz, qw = self.yaw_to_quaternion(desired_yaw)
            
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.navigator.get_clock().now().to_msg()
            
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            
            poses.append(pose)
            self.get_logger().info(f'등록된 웨이포인트: X({x:.2f}m), Y({y:.2f}m) / Yaw: {desired_yaw}도')
            
        return poses

    def set_initial_pose(self):
        """맵 상의 절대 좌표 기준으로 내 로봇이 현재 어디있는지 쐐기를 박는 함수"""
        qz, qw = self.yaw_to_quaternion(self.initial_yaw)
        
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = float(self.initial_x)
        initial_pose.pose.position.y = float(self.initial_y)
        initial_pose.pose.orientation.x = 0.0
        initial_pose.pose.orientation.y = 0.0
        initial_pose.pose.orientation.z = qz
        initial_pose.pose.orientation.w = qw
        
        self.navigator.setInitialPose(initial_pose)
        self.get_logger().info(f'초기 위치 셋팅 완료: 맵 기준 X({self.initial_x}), Y({self.initial_y})')

    def start_patrolling(self):
        self.get_logger().info('Nav2 시스템 활성화 기동 대기 중...')
        self.navigator.waitUntilNav2Active()

        waypoints = self.generate_poses()
        cycle_count = 0
        
        while rclpy.ok():
            cycle_count += 1
            self.get_logger().info(f'--- 순찰 {cycle_count}회차 시작 ---')
            
            # 셋팅된 좌표들을 관통하며 자연스런 주행 시작 (goThroughPoses)
            self.navigator.goThroughPoses(waypoints)

            while not self.navigator.isTaskComplete():
                pass 

            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f'순찰 {cycle_count}회차 완료! 다음 사이클 진행을 준비합니다.')
            elif result == TaskResult.CANCELED:
                self.get_logger().warn('순찰이 캔슬되었습니다.')
                break
            elif result == TaskResult.FAILED:
                self.get_logger().error('주행에 실패했습니다.')
                break

def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    node.start_patrolling()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
