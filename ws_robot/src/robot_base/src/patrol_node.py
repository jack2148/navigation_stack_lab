#!/usr/bin/env python3
import math
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose2D
from std_msgs.msg import String, Empty
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')
        
        # ---------- [추가] 로봇이 켜질 때 고정된 시작 기준 좌표 (초기 위치) ----------
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.initial_yaw = 0.0

        # ---------- 리스트 하나씩 이동 및 캡처를 위한 변수 ----------
        self.sorted_waypoints = []
        self.current_wp_index = 0
        self.direction = 1 # 1은 순방향(정순), -1은 역방향(역순)
        
        self.navigator = BasicNavigator()

        # ---------- GUI JSON 통신(Subscriber & Publisher) 설정 ----------
        self.subscription = self.create_subscription(
            String,
            '/patrol/waypoints_json',
            self.waypoints_callback,
            10
        )
        self.control_sub = self.create_subscription(
            String,
            '/patrol/command',
            self.control_callback,
            10
        )
        self.capture_pub = self.create_publisher(
            String,
            '/patrol/capture_trigger',
            10
        )
        self.camera_sub = self.create_subscription(
            String,
            '/patrol/capture_done',
            self.camera_callback,
            10
        )
        
        self.get_logger().info('GUI Waypoints 구독 시작: /patrol/waypoints_json 토픽')
        self.get_logger().info('GUI Control 구독 시작: /patrol/command 토픽')
        self.get_logger().info('Capture Trigger 발행 시작: /patrol/capture_trigger 토픽')
        self.get_logger().info('Camera Control 구독 시작: /patrol/capture_done 토픽 (done 대기용)')
        
        # ---------- 상태 지속 퍼블리시 및 갱신용 노드 추가 ----------
        self.goal_pose_pub = self.create_publisher(Pose2D, '/goal_pose_2d', 10)
        self.next_place_pub = self.create_publisher(String, '/next_place_id', 10)
        self.status_pub = self.create_publisher(String, '/robot_status', 10)
        self.reload_pub = self.create_publisher(Empty, '/patrol/reload_waypoints', 10)

        # 주행 상태 확인 전용 변수
        self.check_timer = None
        self.is_running = True
        self.is_returning_home = False  # 복귀 중인지 판단하는 상태 플래그
        self.is_waiting_for_waypoints = True # [신규] 웨이포인트 수신 대기 무한 루프 플래그 (초기 켜짐)

        # 1초 주기로 무조건 쏘던 타이머는 제거하고 이벤트 발생 시 배출하도록 변경
        # self.status_emit_timer = self.create_timer(1.0, self.publish_continuous_status)
        
        # JSON을 받을 때까지 주기적으로(3초마다) 서버를 두드리는 핑(Ping) 타이머 가동
        self.reload_ping_timer = self.create_timer(3.0, self._publish_reload_continuous)

    def _publish_reload_continuous(self):
        """플래그가 켜져 있는 동안 무한히 서버 측에 웨이포인트를 달라고 핑을 보냅니다"""
        if self.is_waiting_for_waypoints:
            self.get_logger().info('>>> 웨이포인트(JSON)가 아직 없습니다. /patrol/reload_waypoints 트리거를 지속 핑(Ping) 요청합니다!')
            self.reload_pub.publish(Empty())

    def publish_current_status(self):
        """명령, 갱신, 도달 등 '상태가 변경되었을 때만' 1회성으로 호출되는 퍼블리셔"""
        # 1. /robot_status 상태 전송
        status_msg = String()
        if self.is_returning_home:
            status_msg.data = "충전소 복귀 중"
        elif not self.is_running:
            status_msg.data = "순찰 일시 정지"
        else:
            status_msg.data = "순찰 중"
        self.status_pub.publish(status_msg)

        # 2. /goal_pose_2d 와 /next_place_id 전송
        goal_msg = Pose2D()
        id_msg = String()
        
        if self.is_returning_home:
            id_msg.data = "충전소(Origin)"
            goal_msg.x = 0.0
            goal_msg.y = 0.0
            goal_msg.theta = 0.0
            self.next_place_pub.publish(id_msg)
            self.goal_pose_pub.publish(goal_msg)
            
        elif self.sorted_waypoints and self.current_wp_index < len(self.sorted_waypoints):
            wp = self.sorted_waypoints[self.current_wp_index]
            id_msg.data = wp.get("place_id", "Unknown")
            goal_msg.x = float(wp.get("x", 0.0))
            goal_msg.y = float(wp.get("y", 0.0))
            goal_msg.theta = float(wp.get("yaw", 0.0))
            
            self.next_place_pub.publish(id_msg)
            self.goal_pose_pub.publish(goal_msg)

    def yaw_to_quaternion(self, yaw_degree):
        yaw_rad = math.radians(yaw_degree)
        qz = math.sin(yaw_rad / 2.0)
        qw = math.cos(yaw_rad / 2.0)
        return qz, qw


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

    def camera_callback(self, msg):
        """카메라 촬영 및 서버 송신 완료 후 발송되는 명령 수신부"""
        try:
            status, received_place_id = msg.data.strip().split(":", 1)
            status = status.strip().lower()
            received_place_id = received_place_id.strip()
        except ValueError:
            self.get_logger().error(f'잘못된 카메라 통신 포맷입니다 ("status:place_id" 형태 필요): {msg.data}')
            return
            
        if status == 'done':
            if not self.is_running:
                self.get_logger().warn(f'카메라 완료({received_place_id}) 신호가 왔으나, 로봇 제어가 일시 정지(Pause) 상태라 대기합니다.')
                return
                
            # 현재 대기중인 웨이포인트의 place_id와 일치하는지 교차 검증 보완
            if self.sorted_waypoints and self.current_wp_index < len(self.sorted_waypoints):
                current_place_id = self.sorted_waypoints[self.current_wp_index].get("place_id", "Unknown")
                if received_place_id != current_place_id:
                    self.get_logger().warn(f'수신된 완료 ID({received_place_id})가 현재 목표({current_place_id})와 다릅니다! 하지만 일단 진행합니다.')

            self.get_logger().info(f'>>> [촬영 완료] {received_place_id} 처리 성공! 다음 인덱스로 순차 이동할게요!')
            self.current_wp_index += self.direction
            self.publish_current_status() # [상태 변경] 다음 타겟으로 갱신되었으므로 1회 송출
            self.start_patrolling_async()
        else:
            self.get_logger().info(f'카메라에서 done이 아닌 다른 상태를 보냈습니다: {status}')

    def control_callback(self, msg):
        """GUI에서 제어 신호가 올 때 호출됩니다."""
        cmd = msg.data.strip().lower()
        if cmd == 'pause_patrol':
            self.get_logger().warn('>>> [PAUSE] 명령 수신! 즉시 일반 순찰을 정지합니다.')
            self.is_running = False
            self.navigator.cancelTask()
            self.publish_current_status() # [상태 변경] 정지 송출
        elif cmd == 'start_patrol':
            if not self.sorted_waypoints:
                self.get_logger().error('>>> [START] 명령 수신. 하지만 지정된 웨이포인트가 없어 출발할 수 없습니다.')
                return
            self.get_logger().info('>>> [START] 명령 수신! 일반 순찰을 이어서 시작/재개합니다.')
            self.is_running = True
            self.is_returning_home = False
            self.publish_current_status() # [상태 변경] 순찰 중 송출
            self.start_patrolling_async()
        elif cmd == 'return_to_charge' or cmd == 'retrun_to_charge':
            self.get_logger().info('>>> [RETURN] 복귀 명령 수신! 충전소(0,0,0)로 복귀합니다.')
            self.is_running = False
            self.is_returning_home = True
            self.publish_current_status() # [상태 변경] 복귀 중 송출
            self.navigator.cancelTask()
            self.return_to_origin()
        else:
            self.get_logger().info(f'알 수 없는 제어 명령입니다: {cmd}')

    def return_to_origin(self):
        """원점(0,0,0)으로 단일 이동 명령을 전송합니다."""
        self.navigator.waitUntilNav2Active()
        qz, qw = self.yaw_to_quaternion(0.0)
        
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        
        self.get_logger().info('원점 복귀 주행을 요청했습니다.')
        self.navigator.goToPose(pose)
        
        if self.check_timer is not None:
            self.check_timer.cancel()
        self.check_timer = self.create_timer(1.0, self.check_navigation_status)

    def waypoints_callback(self, msg):
        """GUI에서 전송한 새로운 포맷 JSON 수신 및 순차 로직"""
        self.get_logger().info('GUI 웨이포인트 JSON 수신됨')
        
        # [방어 로직] 진행 중이던 기존 주행 목표를 깔끔하게 파기(Abort)하여 충돌 방지
        if not self.navigator.isTaskComplete():
            self.get_logger().info('기존 주행 태스크가 발견되어 초기화 취소(Cancel)를 진행합니다.')
            self.navigator.cancelTask()
            
        try:
            data = json.loads(msg.data)
            places = data.get("places", [])
            
            if not places:
                self.get_logger().error('JSON 데이터 내부에 "places" 배열이 비어있습니다.')
                return
                
            # JSON 내의 patrol_order 값 기준으로 배열을 오름차순 정렬합니다.
            places.sort(key=lambda wp: int(wp.get("patrol_order", 0)))
            
            self.sorted_waypoints = places
            self.current_wp_index = 0
            self.direction = 1 # 새 리스트를 받으면 늘 정방향으로 시작
            self.is_running = True
            self.is_returning_home = False
            self.is_waiting_for_waypoints = False # [상태 변경] 무한 핑(Ping) 종료
            
            self.publish_current_status() # [상태 변경] 맵 리로드 후 0번 타겟 송출
            self.get_logger().info(f'총 {len(self.sorted_waypoints)}개의 웨이포인트 정렬 성공. 순차 주행 액션을 시작합니다.')
            self.start_patrolling_async()
            
        except json.JSONDecodeError:
            self.get_logger().error('JSON 파싱 에러: 올바른 JSON 포맷이 아닙니다.')
        except Exception as e:
            self.get_logger().error(f'웨이포인트 처리 중 예상치 못한 에러: {e}')

    def start_patrolling_async(self):
        """현재 순서(index)의 단일 웨이포인트로 목적지 설정 및 발차"""
        if not self.sorted_waypoints:
            self.get_logger().warn('등록된 로컬 웨이포인트가 없습니다!')
            return
            
        n_wps = len(self.sorted_waypoints)
        # 배열을 넘기지 않기 위해 정방향/역방향 양쪽 끝단 초과 여부를 감지하여 방향 전환(핑퐁)
        if self.current_wp_index >= n_wps:
            if self.is_running:
                self.get_logger().info('마지막 타겟 도달 완료! 역순으로 주행 방향을 뒤집어 되돌아갑니다.')
                self.direction = -1
                self.current_wp_index = n_wps - 2
                
                if self.current_wp_index < 0:
                    self.current_wp_index = 0
            else:
                return
        elif self.current_wp_index < 0:
            if self.is_running:
                self.get_logger().info('>>> [왕복 순회 완료] 첫 지점(1번)으로 모두 되돌아왔습니다! 서버에 새로운 맵(reload)을 무한 요청합니다.')
                
                # 새 웨이포인트(JSON)가 통신으로 도달할 때까지 로봇의 엔진을 내리고 무한 핑(Ping) 모드 진입
                self.is_running = False
                self.is_waiting_for_waypoints = True
                
                self.reload_pub.publish(Empty()) # 즉시 1회 발포
                self.publish_current_status()    # [상태 변경] 대기 모드 진입 송출
                return
            else:
                return

        wp = self.sorted_waypoints[self.current_wp_index]
        place_id = wp.get("place_id", "Unknown")
        target_x = float(wp.get("x", 0.0))
        target_y = float(wp.get("y", 0.0))
        target_yaw = float(wp.get("yaw", 0.0))
        
        qz, qw = self.yaw_to_quaternion(target_yaw)
        
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = target_x
        pose.pose.position.y = target_y
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        
        self.get_logger().info(f'-> [출발] {self.current_wp_index+1}번째 타겟({place_id}) 이동 중...')
        
        # [수정] 콜드 부팅 시 첫 목표가 무시되지 않도록 Nav2 준비 보장
        self.navigator.waitUntilNav2Active()
        
        # goThroughPoses대신 단일 목적지 goToPose를 호출해 각각 멈추도록 유도
        self.navigator.goToPose(pose)
        
        if self.check_timer is not None:
            self.check_timer.cancel()
        self.check_timer = self.create_timer(1.0, self.check_navigation_status)

    def check_navigation_status(self):
        """단일 지점 도착 여부 체크"""
        if self.navigator.isTaskComplete():
            self.check_timer.cancel()
            self.check_timer = None
            
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                # 1. 원점 복귀 성공 분기
                if self.is_returning_home:
                    self.get_logger().info('>>> [복귀 완료] 원점(충전소) 복귀가 성공적으로 완료되었습니다. 명령을 대기합니다.')
                    self.is_returning_home = False  # 안전을 위해 상태 해제
                    self.publish_current_status()   # [상태 변경] 대기(일시 정지) 강등 상태 송출
                    return

                # 2. 일반 목적지 순찰 성공 분기
                if not self.sorted_waypoints:
                    return

                completed_wp = self.sorted_waypoints[self.current_wp_index]
                place_id = completed_wp.get("place_id", "Origin_or_Unknown")
                
                self.get_logger().info(f'>>> [완벽 도착] {place_id} 위치에 부드럽게 정차했습니다.')
                
                # 2. 도착 후 사진 캡처용 트리거 퍼블리시
                trigger_msg = String()
                trigger_msg.data = place_id
                self.capture_pub.publish(trigger_msg)
                self.get_logger().info(f'-> [발행됨] 캡처 트리거 이벤트 전송 완료: {place_id}')
                self.get_logger().info(f'>>> 카메라 측의 /patrol/capture_done ("done:{place_id}") 통신을 무기한 대기합니다...')
                
                # 기존 로직: 도착 즉시 current_wp_index += 1 후 출발이었으나, 
                # 이제는 여기서 완전히 멈춰서 무한 대기하며 camera_callback이 호출될 때까지 대기합니다.
                    
            elif result == TaskResult.CANCELED:
                self.get_logger().warn('주행이 중도 취소되거나 Pause 되었습니다.')
            elif result == TaskResult.FAILED:
                self.get_logger().error('주행 중 치명적인 문제가 발생해 이동 실패했습니다.')

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
