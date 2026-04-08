# Sensor Integration Issues

## 1. LiDAR checksum error

### Problem
LiDAR 실행 시 checksum error가 발생하며 정상적으로 scan 데이터가 수신되지 않음

### Cause
- 전력 공급 부족으로 인해 LiDAR 데이터 전송이 불안정
- USB 포트 권한 설정 문제로 장치 접근 제한

### Solution
- 안정적인 전원 공급을 위해 전원 라인 점검 및 개선
- `/dev/ttyUSB*` 권한 수정 (chmod 또는 udev rule 설정)

### Result
LiDAR scan 데이터가 정상적으로 수신되며 checksum error가 발생하지 않음

### Insight
센서 통신 문제는 소프트웨어뿐 아니라 전력/권한 문제일 가능성이 높으므로 하드웨어 상태를 함께 점검해야 한다

---

## 2. LiDAR 방향 반전 문제

### Problem
LiDAR를 반대로 장착하여 좌우 방향이 반전된 scan 데이터 발생

### Cause
센서 장착 방향과 소프트웨어 설정 불일치

### Solution
LiDAR 드라이버 파라미터에서 `inverted = true` 설정

### Result
좌우 방향이 정상적으로 보정됨

### Insight
센서 물리적 장착 방향과 좌표계 정의는 반드시 일치해야 한다
