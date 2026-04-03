# Mapping Issues

## SLAM map distortion (맵 깨짐 문제)

### Problem
SLAM 수행 시 생성된 맵이 왜곡되거나 깨져서 나타남

### Cause
복합적인 원인으로 발생:
- LiDAR 방향 반전
- Encoder 부호 오류
- LiDAR와 TF 간 timestamp 불일치

### Solution
- LiDAR 방향 보정 (`inverted = true`)
- encoder 부호 수정
- SLAM 실행 순서 조정 (LiDAR 및 TF 안정화 후 실행)
- encoder delay (~0.25초) 보정하여 timestamp alignment 수행

### Result
맵이 정상적으로 생성되고 geometry consistency 확보

### Insight
SLAM 문제는 단일 원인이 아닌 경우가 많으며, 센서 방향/부호/시간 동기화가 동시에 맞아야 정상 동작한다
