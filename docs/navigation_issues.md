# Navigation Issues

## 1. 주행 중 진동 (oscillation)

### Problem
로봇이 목표 지점으로 이동하는 과정에서 좌우로 흔들리는 진동 발생

### Cause
목표 지점(goal)에 대한 weight가 너무 높아 로봇이 각도를 과도하게 맞추려는 현상

### Solution
- goal 관련 gain 감소
- obstacle 관련 gain 증가

### Result
진동이 감소하고 보다 안정적인 경로 생성

### Related Parameters
- goal_distance_bias
- path_distance_bias
- obstacle_scale

### Insight
Nav2 local planner tuning에서 goal/obstacle balance가 매우 중요하며, 과도한 goal bias는 oscillation을 유발한다

---

## 2. Mapping 후 주행 불가 (장애물 잔상)

### Problem
Waypoint를 발행했음에도 로봇이 출발하지 못함

### Cause
Mapping 과정에서 실제 구조물(기둥 등)이 costmap에 지속적으로 장애물로 남아 있음

### Solution
- `laser_filter` 라이브러리를 사용하여 0.35m 이하 scan 데이터 제거
- obstacle radius 감소

### Result
불필요한 장애물 제거 후 정상 주행 가능

### Related Parameters
- obstacle_range
- inflation_radius
- laser_filter threshold

### Insight
Costmap에 남아있는 noise 또는 구조물은 navigation failure의 주요 원인이므로 filtering 전략이 중요하다
