# TF and Frame Issues

## TF frame mismatch (base_link vs base_footprint)

### Problem
TF tree가 일관되지 않아 localization 및 navigation이 불안정

### Cause
일부 노드는 `base_link`, 일부는 `base_footprint` 기준으로 설정되어 frame 불일치 발생

### Solution
모든 TF 기준을 `base_footprint`로 통일

### Result
TF tree가 정상적으로 연결되며 navigation 안정화

### Insight
ROS2 시스템에서 frame_id 불일치는 전체 시스템 오류로 이어지므로 초기 설계에서 반드시 통일해야 한다
