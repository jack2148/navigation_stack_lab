#!/usr/bin/env python3
import math
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')
        
        # ---------- [추가] 로봇이 켜질 때 고정된 시작 기준 좌표 (초기 위치) ----------
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.initial_yaw = 0.0

        # ---------- 변경: 하드코딩된 변수 빈 배열로 초기화 ----------
        self.waypoints_world = []
        
        self.navigator = BasicNavigator()

        # ---------- GUI JSON 통신(Subscriber) 설정 ----------
        self.subscription = self.create_subscription(
            String,
            '/gui/waypoints',
            self.waypoints_callback,
            10
        )
        self.get_logger().info('GUI Waypoints 구독 시작: /gui/waypoints 토픽에서 대기 중...')
        
        # 주행 상태 확인을 위한 비동기 타이머 변수
        self.check_timer = None

    def yaw_to_quaternion(self, yaw_degree):
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
            
        return poses

    def set_initial_pose(self):
        """맵 상의 절대 좌표 기준으로 내 로봇이 현재 어디있는지 세팅"""
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

    def waypoints_callback(self, msg):
        """GUI에서 std_msgs/String으로 전송한 JSON 메세지 수신 및 파싱 코어 로직"""
        self.get_logger().info(f'GUI 웨이포인트(JSON) 수신됨: {msg.data}')
        try:
            # JSON 파싱
            data = json.loads(msg.data)
            
            # 들어온 데이터가 배열 형식인지 단일 객체인지 검사 후 배열로 통일
            if isinstance(data, dict):
                data = [data]
                
            new_waypoints = []
            for wp in data:
                # 필수 필드가 보장되어 있는지 확인 (x, y, yaw)
                new_waypoints.append((float(wp['x']), float(wp['y']), float(wp['yaw'])))
            
            self.waypoints_world = new_waypoints
            self.get_logger().info(f'총 {len(self.waypoints_world)}개의 웨이포인트 등록 성공. 비동기 주행을 시작합니다.')
            
            # 파싱 및 등록 완료되었으므로 바로 구동 명령
            self.start_patrolling_async()
            
        except json.JSONDecodeError:
            self.get_logger().error('JSON 파싱 에러: GUI 데이터가 올바른 JSON 문자열 포맷이 아닙니다.')
        except KeyError as e:
            self.get_logger().error(f'파라미터 누락 에러: {e}. "x", "y", "yaw" 키가 JSON 내에 반드시 포함되어야 합니다.')
        except Exception as e:
            self.get_logger().error(f'웨이포인트 처리 중 예상치 못한 에러: {e}')

    def start_patrolling_async(self):
        """블로킹 없이 비동기적으로 Nav2에 순찰 명령을 내리는 함수"""
        if not self.waypoints_world:
            self.get_logger().warn('등록된 웨이포인트가 없습니다!')
            return

        waypoints = self.generate_poses()
        self.get_logger().info('Nav2 서버 (goThroughPoses)로 목표 타겟 리스트를 전송합니다...')
        
        # 무한 루프 대신 1회 비동기 액션 목표 전송
        self.navigator.goThroughPoses(waypoints)
        
        # 액션 골(목표)이 전송되었으니 진행 상황을 스레드를 멈추지 않고 확인하기 위해 타이머 생성
        if self.check_timer is not None:
            self.check_timer.cancel()
        self.check_timer = self.create_timer(1.0, self.check_navigation_status)

    def check_navigation_status(self):
        """1초에 한번씩 콜백 방식으로 주행 완료 여부 체크"""
        if self.navigator.isTaskComplete():
            self.check_timer.cancel()
            self.check_timer = None
            
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info('모든 GUI 웨이포인트 순찰 주행을 완벽하게 완료했습니다! 다음 데이터를 기다립니다.')
            elif result == TaskResult.CANCELED:
                self.get_logger().warn('주행이 중도 캔슬되었습니다.')
            elif result == TaskResult.FAILED:
                self.get_logger().error('주행 중 치명적인 문제가 발생해 실패했습니다.')

def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    
    # 노드는 무한 루프(while True) 없이 spin 모드로 진입. 
    # 콜백 리스너가 백그라운드에서 토픽을 계속 모니터링할 수 있도록 보장.
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('사용자에 의해 종료...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
