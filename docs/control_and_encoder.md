# Control & Encoder Issues

## 1. cmd_vel publish 불가 (Twist vs TwistStamped)

### Problem
Wheel encoder 기반으로 cmd_vel을 발행하려 했으나 메시지 타입 mismatch로 publish 실패

### Cause
Nav2 또는 시스템 설정에서 `geometry_msgs/TwistStamped` 사용 중이었음

### Solution
`use_stamped_vel = false` (또는 관련 unstamped 파라미터) 설정하여 `geometry_msgs/Twist` 사용

### Result
cmd_vel 정상 publish 가능

### Insight
ROS2에서는 메시지 타입(Twist vs TwistStamped)이 시스템 전체와 일관되어야 한다

---

## 2. Encoder 부호 오류 (CAN 통신)

### Problem
로봇 주행 시 좌우 바퀴 방향이 다르게 동작하여 정상적인 주행 불가

### Cause
CAN 통신으로 전달되는 encoder 값이 (+, -)가 아닌 (+, +) 형태로 잘못 설정됨

### Solution
`can_hw` 코드에서 encoder 부호를 올바르게 수정

### Result
좌우 바퀴 방향이 정상적으로 동작

### Insight
Encoder 부호 오류는 odometry 전체를 망가뜨리는 치명적인 문제이므로 초기 검증 필수
