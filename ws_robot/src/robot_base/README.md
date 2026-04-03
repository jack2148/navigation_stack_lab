# Robot Base Execution Protocol

본 패키지는 로봇의 기본 구동, SLAM(지도 작성), Nav2(자율 주행)를 지원함
아래의 절차에 따라 터미널을 열고 워크스페이스를 빌드/소싱한 뒤 실행

## 0. 공통 사항 (빌드 및 소싱)
가장 먼저 워크스페이스(`ws_robot`) 폴더로 이동하여 패키지를 빌드하고 ROS 2 환경 소싱
모든 새로운 터미널을 열 때마다 반드시 소싱(`source`) 명령어를 실행

```bash
cd ~/navigation_stack_lab/ws_robot
colcon build --symlink-install
source install/setup.bash
```

---

## 1. SLAM 프로토콜 (지도 작성)
새로운 환경에서 지도를 작성(Mapping)할 때 사용하는 실행 프로토콜

### [터미널 1] 로봇 하드웨어 및 기본 노드 구동
로봇의 하드웨어(라이다, 모터 제어기 등)를 활성화하고 기본 변환(TF)을 실행
```bash
ros2 launch robot_base bringup.launch.py
```

### [터미널 2] SLAM 활성화
하드웨어가 정상적으로 구동된 것을 확인한 뒤, SLAM 노드 실행
```bash
ros2 launch robot_base slam.launch.py
```

### [터미널 3] 지도 저장 (완료 시)
지도가 완성되면 새로운 터미널을 열고 아래 명령어로 맵 저장
```bash
ros2 run nav2_map_server map_saver_cli -f ~/navigation_stack_lab/ws_robot/src/robot_base/maps/my_map --ros-args -p save_map_timeout:=10000
```

---

## 2. Nav2 프로토콜 (자율 주행)
저장되어 있는 지도를 기반으로 자율 주행 알고리즘을 켤 때 사용하는 프로토콜

### [터미널 1] 로봇 하드웨어 구동
SLAM 때와 완전히 동일하게 기본 하드웨어 실행.
```bash
ros2 launch robot_base bringup.launch.py
```

### [터미널 2] Nav2 내비게이션 실행
Nav2 코어 노드들을 실행합니다. 
> **참고**: 이 파일을 실행하면 백그라운드에서 실시간 좌표를 뽑아내는 `robot_pose_tf_bridge.py` 노드도 10Hz 속도로 묶여서 연동되어 자동 실행
```bash
ros2 launch robot_base nav2.launch.py
```

### [터미널 3] 패트롤 동작 등 부가기능 (선택 사항)
순찰 기능 파이썬 스크립트 등 목적지 이동 명령을 별도로 실행하고 싶을 때 사용합니다.
```bash
ros2 launch robot_base patrol.launch.py
```

### 💡 [초기 위치 설정 및 테스트 방향]
1. `rviz2` 혹은 자체 구동 중인 UI/GUI를 연동
2. 실행 직후에는 로봇이 자기 위치를 모름. **2D Pose Estimate** 기능을 이용해 도면 상에서 로봇이 현재 바라보고 있는 위치와 각도를 대략적으로 찍어줌. (AMCL 파티클 초기화 단계)
3. 레이저 스캔 선(빨간 점들)이 지도 벽면과 얼추 맞물리는지 확인
4. **Nav2 Goal (2D Goal Pose)** 기능을 눌러 목적지를 찍어주면 매끄럽게 회피하며 주행
