# MPPI 튜닝 시행착오 기록

## 개요

Nav2 MPPI Controller 기반 순찰 로봇 주행 과정에서 발생한 문제들과 해결 과정을 기록합니다.
적용 파일: `config/mppi_params.yaml`, `config/mppi_coordi.yaml`

---

## 문제 1: 트래킹 종료 후 waypoint 주행 시 목적지 방황 반복

### 증상
- patrol 실행 중 사람 감지 → tracking 모드 전환 (cmd_vel 직접 제어)
- tracking 종료(IDLE) 후 waypoint 주행 재개 시 목적지 근처에서 계속 방황하며 주행 반복

### 원인 분석

**1. AMCL 로컬라이제이션 드리프트 (주 원인)**
- tracking 중 Nav2를 완전히 우회하여 `/diff_drive_controller/cmd_vel`에 직접 발행
- AMCL은 계속 동작하지만 CPU 부하로 수렴이 늦어짐
- IDLE 전환 시 즉시 `start_patrolling_async()` 호출 → AMCL이 수렴하기 전에 MPPI가 stale map pose 기준으로 경로 계산 → 목적지 근처 oscillation

**2. `stateful: true` goal checker**
- `cancelTask()` 후 빠르게 새 `goToPose()` 전송 시 goal checker 내부 상태가 남아 혼란 발생

**3. velocity_smoother 상태 불일치**
- `feedback: "OPEN_LOOP"` 모드에서 tracking 중 직접 cmd_vel 발행 시 smoother가 인지하지 못함
- Nav2 재개 시 smoother 내부 속도 추정값과 실제 속도 불일치

**4. MPPI CPU 과부하**
- `batch_size: 800`, `time_steps: 30`, `controller_frequency: 10Hz`
- CPU 부하가 높을 때 10Hz 제어 루프를 못 맞춰 overt/undershoot 발생

### 해결 방향
- IDLE 전환 시 즉시 재개하지 않고 타이머 딜레이 + `clearLocalCostmap()` 호출 → **적용 완료 (아래 참조)**
- `stateful: false` 적용 검토
- MPPI 파라미터 축소로 CPU 부하 완화 (아래 참조)

---

## 문제 2: 일반 순찰(트래킹 없음) 중에도 목적지 반복 주행

### 증상
- tracking 없이 일반 waypoint 순찰 중에도 목적지 근처에서 oscillation 후 도달 실패 반복

### 원인
- AMCL 드리프트, cancelTask() 등 외부 요인 없음
- **MPPI CPU 과부하로 제어 주기 저하**가 직접 원인으로 확인
  - 실제 제어 주기가 5~6Hz로 떨어짐 → cmd_vel 지연 → 오버슈트
  - goal tolerance(0.1m) 밖으로 벗어남 → MPPI가 다시 전진 → 반복
  - `isTaskComplete()`가 영원히 false → `check_timer` 무한 반복

### 확인 방법
```bash
# 실제 제어 주기 확인
ros2 topic hz /diff_drive_controller/cmd_vel
# 10Hz가 아닌 5~6Hz 나오면 CPU 과부하 확정

# CPU 사용량 확인
htop
```

---

## MPPI 파라미터 변경 이력

### 변경 1: CPU 부하 완화

| 항목 | 변경 전 | 변경 후 | 이유 |
|---|---|---|---|
| `batch_size` | 800 | 500 | CPU 부하 약 35% 감소 |
| `time_steps` | 30 | 25 | trajectory 계산량 감소 |
| `controller_frequency` | 10.0 Hz | 8.0 Hz | 실제 가능한 주기로 낮춰 안정화 |

> batch_size 500 + time_steps 25 = 12,500개 trajectory point 평가
> 실내 환경(장애물 적음)에서는 800개와 체감 차이 거의 없음
> 오히려 제어 주기가 안정적으로 유지되어 주행이 더 부드럽게 느껴짐

### 변경 2: 속도 상향

| 항목 | 변경 전 | 변경 후 | 이유 |
|---|---|---|---|
| `vx_max` | 0.15 m/s | 0.4 m/s | 주행 속도 너무 느려 상향 |
| `velocity_smoother max_velocity` | [0.15, 0.0, 0.3] | [0.4, 0.0, 0.8] | vx_max에 맞춰 동기화 |
| `velocity_smoother min_velocity` | [-0.15, 0.0, -0.3] | [-0.4, 0.0, -0.8] | vx_max에 맞춰 동기화 |

---

## goal_tolerance 관련 메모

- `xy_goal_tolerance: 0.1`, `yaw_goal_tolerance: 0.1` 고정
- 사진 촬영 판별 정밀도 요구사항으로 인해 완화 불가
- CPU 과부하 시 0.1m tolerance에서 oscillation 발생 가능 → batch_size 축소로 대응

---

## patrol_node.py 코드 변경 이력

### IDLE 전환 시 AMCL 수렴 대기 (2초) 적용

**문제**: IDLE 상태로 전환될 때 `start_patrolling_async()`를 즉시 호출하면 AMCL이 수렴하기 전에 MPPI가 재개되어 목적지 근처 oscillation 발생

**추가 고려사항**: IDLE 상태가 반복적으로 들어올 경우 `create_timer()`가 중복 생성되어 `_resume_patrol_once()`가 여러 번 호출되는 문제 → 타이머 참조를 멤버 변수(`_resume_timer`)로 관리하여 해결

**해결 방식**:
```python
# IDLE 진입 시
if self._resume_timer is not None:
    self._resume_timer.cancel()       # 기존 타이머 취소 (중복 방지)
self._resume_timer = self.create_timer(2.0, self._resume_patrol_once)

# 콜백 내부
def _resume_patrol_once(self):
    self._resume_timer.cancel()       # one-shot 처리
    self._resume_timer = None
    self.navigator.clearLocalCostmap()  # MPPI 상태 초기화
    self.start_patrolling_async()
```

**동작 흐름**:
```
IDLE 첫 번째 진입 → 타이머 생성 (2초 카운트 시작)
IDLE 두 번째 진입 → 기존 타이머 cancel() → 새 타이머 생성 (2초 리셋)
2초 후 콜백 실행  → 타이머 cancel() + None 처리 → clearLocalCostmap() → 순찰 재개
```

**AMCL 수렴 시간 기준**:

| 상황 | 수렴 시간 |
|---|---|
| 거의 안 움직인 경우 | 0.5 ~ 1초 |
| 천천히 이동 후 정지 | 1 ~ 2초 |
| 빠르게 이동 후 정지 | 2 ~ 3초 |
| 급격한 방향 전환 포함 | 3 ~ 5초 |

현재 시스템(CPU 부하 있음 + 빠른 tracking 이동) 기준으로 **2초 딜레이** 적용

---

## 문제 3: controller_server 실행 실패 (costmap 미표시)

### 증상
- `bringup_nav` 실행 후 RViz2에서 초기 포즈 설정해도 costmap이 뜨지 않음
- `ros2 topic list | grep costmap` 결과 토픽 없음
- 터미널에 `Failed to bring up all requested nodes. Aborting bringup` 출력

### 원인
`controller_frequency`를 10Hz → 8Hz로 낮추면서 `model_dt`를 그대로 0.1로 둔 것이 원인.

MPPI는 내부적으로 아래 조건을 검증합니다:
```
model_dt >= controller_period (= 1 / controller_frequency)
```

| 항목 | 값 |
|---|---|
| `controller_frequency` | 8.0 Hz |
| controller period | 1 / 8.0 = **0.125초** |
| `model_dt` | 0.1초 |
| 조건 | 0.1 >= 0.125 → **위반** |

### 해결
`model_dt`를 `1 / controller_frequency`에 맞춰 수정:

| 항목 | 전 | 후 |
|---|---|---|
| `model_dt` | 0.1 | 0.125 |

> `controller_frequency`를 변경할 때는 반드시 `model_dt = 1 / controller_frequency` 로 맞춰야 합니다.

---

## 추가 튜닝 여지

- `batch_size`를 400까지 낮출 수 있으나 복잡한 장애물 환경에서는 경로 품질 저하 가능
- `stateful: false` 적용 시 tracking 후 재개 안정성 추가 향상 기대
