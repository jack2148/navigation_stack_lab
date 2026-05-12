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

## 2. LiDAR 타임스탬프 불일치 (`TF_OLD_DATA` 경고)

### Problem
Nav2 / SLAM 실행 시 아래 경고가 반복되며 scan 데이터가 간헐적으로 무시됨:
```
TF_OLD_DATA ignoring data from the past for frame /laser_frame
```

### Cause
YDLidar 드라이버가 발행하는 `/scan` 타임스탬프가 ROS 시스템 클럭과 미세하게 불일치.
드라이버 내부에서 센서 측정 시각 기준으로 stamp를 찍다 보니 시스템 클럭보다 수십~수백 ms 뒤처지는 경우 발생.
AMCL과 costmap은 최신 stamp 기준으로 동작하므로 오래된 stamp의 scan을 버림.

### Solution
`scan_restamper.py` 노드를 추가하여 타임스탬프를 수신 시점 기준으로 교체:
```
YDLidar driver → /scan_pre → scan_restamper (stamp = now()) → /scan → Nav2 / SLAM
```

YDLidar 드라이버의 publish topic을 `/scan_pre`로 변경하고,
`scan_restamper`가 수신 즉시 `header.stamp = now()`로 교체 후 `/scan`으로 재발행.

### Result
`TF_OLD_DATA` 경고 소멸, scan 데이터가 정상적으로 AMCL·costmap에 반영됨.

### Related Files
- `src/scan_restamper.py`
- `config/ydlidar.yaml` — publish topic 설정

---

## 3. LiDAR 방향 반전 문제

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
