# IMU & Pose Discrepancy Issues (시행착오)

## 1. 초기 위치(Initial Pose) 및 Map 좌표계 불일치

### Problem
RViz 상의 초기 로봇 위치가 Map의 실제 좌표계(0,0,0)와 불일치하는 문제

### Cause
로봇 시스템 시작 시 AMCL 및 Odom 프레임의 기준점이 실제 물리적 (0,0,0)이 아닌 위치로 잘못 맵핑되거나 초기화됨

### Solution
로봇의 초기 시작 위치를 Map 좌표계의 Origin (0,0,0) 지점으로 강제 변환(Transform)하도록 로직 구현 및 초기화 스크립트에 반영

### Result
Navigation Stack의 Localization 모듈이 맵의 기준 출발점을 정확하게 인식하게 됨

---

## 2. Waypoint 주행 시 110도 Yaw 뒤틀림

### Problem
조이스틱 컨트롤로 저장한 Waypoint로 이동 명령을 내릴 때, 로봇의 회전 각도(Yaw)가 약 110도 가량 크게 어긋나는 현상 발생

### Cause
- 로봇의 Base Pose 프레임, IMU 로컬 센서 프레임, 그리고 전체 Map 프레임 간의 Transformation(좌표 변환) 과정
- TF Tree 상에서 회전 각도가 다른 축을 기준으로 투영되어 잘못 반영됨

### Solution
- Base 프레임과 IMU, Map 프레임 간의 Coordinate Transformation 로직의 방향성(축 정렬) 점검 및 수정
- Waypoint 목표 지점의 Yaw 값을 계산할 때의 Heading alignment 로직 일치화 설정

### Result
로봇이 Waypoint 도착 시 의도된 관측 방향(Heading)을 정확히 바라보도록 수정 완료

---

## 3. AMCL 추정 포즈와 주행 Odom 간의 약 10도 Yaw 오차 (주행 검증)

### Problem
자율 주행 중 로봇의 실시간 Odometry 추정(`/odometry/filtered` 또는 `/robot_pose`)과 Lidar 기반 AMCL 매칭 위치(`/amcl_pose`) 사이에 지속적으로 약 10도의 Yaw 오차 및 통계적 X/Y 좌표 Offset 발생 

### Cause
- IMU 센서 자체의 고유 기계적 Drift(시간 추이에 따른 누적 오차) 현상
- Odom과 IMU 데이터를 융합하는 `robot_localization` EKF(Extended Kalman Filter) 단계에서 IMU 데이터에 대한 신뢰도(Covariance) 가중치가 현실치와 어긋남

### Solution
- 파이썬 기반 데이터 추출 스크립트(`analyze_true_error.py`)를 개발하여 통제 환경에서 취득한 ROSbag(`error_bag`) 파일들 분석
- AMCL 좌표와 Odom 간의 시간축 동기화 코드를 적용, 최대 에러(Max), 평균 에러(Mean), 오차 추세 그래프(Plotly/Matplotlib)를 산출해 수치적으로 10도의 Yaw 오차를 정량 검증

### Insight & Future Work
IMU는 단기적인 주행 방향성(Heading)을 잡아주지만 장기적으론 Drift가 발현됩니다. 향후 시스템 유지보수를 위해서는 **EKF 파라미터 내 IMU의 Covariance 값을 낮춰 Lidar(AMCL)의 기여도를 끌어올리는 튜닝**을 적용하거나, IMU를 소프트웨어적으로 리셋(Zero-calibration)하는 방법을 파이프라인에 추가하는 것을 권장합니다.
