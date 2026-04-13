#!/usr/bin/env python3
import math
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class ModeNode(Node):
    def __init__(self):
        super().__init__('mode_node')
        
        # ---------- 1. 상태 관리 (State Management) ----------
        self.current_mode = None
        self.waypoints_world = []
        
        self.navigator = BasicNavigator()

        # ---------- 2. 통신 연동 (Subscriber) ----------
        # GUI에서 시간마다 M1, M2 등을 JSON으로 받는 토픽
        self.subscription = self.create_subscription(
            String,
            '/gui/mode_command',
            self.mode_callback,
            10
        )
        # GUI에서 "stop"이나 "go" 명령을 받는 전용 토픽
        self.control_sub = self.create_subscription(
            String,
            '/gui/control',
            self.control_callback,
            10
        )
        self.get_logger().info('모드(Mode) 대기: /gui/mode_command 토픽 대기 중...')
        self.get_logger().info('제어(Control) 대기: /gui/control 토픽(stop/go) 대기 중...')
        
        self.check_timer = None
        self.is_running = True  # 현재 동작 상태 (stop 명령 시 False로 변환)

    def yaw_to_quaternion(self, yaw_degree):
        yaw_rad = math.radians(yaw_degree)
        qz = math.sin(yaw_rad / 2.0)
        qw = math.cos(yaw_rad / 2.0)
        return qz, qw

    def generate_poses(self):
        """좌표 파라미터를 Nav2 액션용 PoseStamped 배열로 포장"""
        poses = []
        for (x, y, desired_yaw) in self.waypoints_world:
            qz, qw = self.yaw_to_quaternion(desired_yaw)
            
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.navigator.get_clock().now().to_msg()
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            
            poses.append(pose)
            
        return poses

    def control_callback(self, msg):
        """GUI에서 정지(stop)나 재개(go) 신호가 올 때 호출됩니다."""
        cmd = msg.data.strip().lower()
        if cmd == 'stop':
            self.get_logger().warn('>>> [STOP] 명령 수신! 즉시 로봇 주행을 정지합니다.')
            self.is_running = False
            self.navigator.cancelTask()
        elif cmd == 'go':
            if not self.waypoints_world:
                self.get_logger().error('>>> [GO] 명령 수신. 하지만 지정된 경로가 없어 출발할 수 없습니다.')
                return
            self.get_logger().info(f'>>> [GO] 명령 수신! [{self.current_mode}] 순찰을 이어서 재개합니다.')
            self.is_running = True
            self.start_mode_patrolling_async()
        else:
            self.get_logger().info(f'알 수 없는 제어 명령입니다: {cmd}')

    def mode_callback(self, msg):
        """
        로직: GUI에서 시간별로 새로운 동작 스케줄(M1, M2 등)이 떨어질 때 호출됩니다.
        설명: 현재 주행 중인 동작을 즉각 취소(덮어쓰기)하고, 새로 들어온 모드의 좌표로 주행을 전환합니다.
        """
        self.get_logger().info('GUI로부터 새로운 모드 명령이 도달했습니다.')
        
        try:
            # 예상되는 JSON 포맷 예:
            # {
            #   "mode": "M1", 
            #   "waypoints": [ {"x":-20.0, "y":-22.4, "yaw":0.0}, {"x":-17.7, "y":-22.4, "yaw":90.0} ]
            # }
            data = json.loads(msg.data)
            
            target_mode = data.get('mode', 'UNKNOWN')
            waypoints_data = data.get('waypoints', [])
            
            if not waypoints_data:
                self.get_logger().warn(f'[{target_mode}] 모드 명령을 받았으나 웨이포인트 데이터가 비어있습니다. 명령 무시.')
                return

            # 중복 모드 명령 수신 방어 (옵션: 필요에 따라 주석 처리 가능)
            # if target_mode == self.current_mode:
            #     self.get_logger().info(f'이미 실행 중인 모드 [{target_mode}] 입니다. 중복 명령을 무시합니다.')
            #     return

            # JSON 안에 있는 웨이포인트들 추출 및 안전 검사
            new_waypoints = []
            for wp in waypoints_data:
                if 'x' in wp and 'y' in wp and 'yaw' in wp:
                    new_waypoints.append((float(wp['x']), float(wp['y']), float(wp['yaw'])))
                else:
                    self.get_logger().warn('일부 웨이포인트에서 필수 필드(x, y, yaw)가 누락되어 제거되었습니다.')

            if not new_waypoints:
                self.get_logger().error(f'[{target_mode}] 모드의 유효한 웨이포인트가 없어 주행 전환에 실패했습니다.')
                return

            # ---------- 3. 데이터 갱신 및 주행 전환 ----------
            self.current_mode = target_mode
            self.waypoints_world = new_waypoints
            self.is_running = True
            
            self.get_logger().info(f'>>> 주행 모드 [{self.current_mode}] (목표 좌표 {len(self.waypoints_world)}개)로 전환 승인 완료!')
            
            # 주소지가 갱신되었으므로, 바로 비동기 주행 실행 (기존 주행은 Nav2에 새 목표를 던짐으로써 자동 Canceled 처리됨)
            self.start_mode_patrolling_async()
            
        except json.JSONDecodeError:
            self.get_logger().error('JSON 디코드 에러: 수신된 데이터가 유효한 텍스트 포맷이 아닙니다.')
        except Exception as e:
            self.get_logger().error(f'모드 변환 과정 중 알 수 없는 에러: {e}')

    def start_mode_patrolling_async(self):
        """블로킹 없이 백그라운드에서 주행 명령을 보내는 로직"""
        if not self.waypoints_world:
            return

        self.navigator.waitUntilNav2Active()
        waypoints = self.generate_poses()
        
        self.get_logger().info(f'[{self.current_mode}] 모드 주행을 곧바로 시작합니다.')
        
        # goThroughPoses를 호출하면 현재 진행 중인 액션(과거 모드)이 있다면 덮어씌워지고 새 좌표로 진행
        self.navigator.goThroughPoses(waypoints)
        
        # 주행 상태 확인을 위한 비동기 타이머 리셋
        if self.check_timer is not None:
            self.check_timer.cancel()
        self.check_timer = self.create_timer(1.0, self.check_navigation_status)

    def check_navigation_status(self):
        if self.navigator.isTaskComplete():
            self.check_timer.cancel()
            self.check_timer = None
            
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f'[{self.current_mode}] 경로 1회전 완료!')
                
                # 정지(stop) 상태가 아니라면 즉시 같은 경로로 무한 반복 출발
                if self.is_running:
                    self.get_logger().info('>>> 휴식 없이 무한 반복 순찰을 재시작합니다.')
                    self.start_mode_patrolling_async()
                else:
                    self.get_logger().info('>>> 정지(Stop) 상태이므로 사이클 종료 후 대기합니다.')
                    
            elif result == TaskResult.CANCELED:
                self.get_logger().warn(f'[{self.current_mode}] 주행이 새로운 명령 수달이나 정지(Stop) 명령에 의해 취소되었습니다.')
            elif result == TaskResult.FAILED:
                self.get_logger().error(f'[{self.current_mode}] 주행 중 문제가 터져 중단되었습니다.')

def main(args=None):
    rclpy.init(args=args)
    node = ModeNode()
    
    try:
        # 백그라운드에서 통신을 계속 대기
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('사용자에 의해 종료...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
