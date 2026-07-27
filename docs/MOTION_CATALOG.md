# Motion Catalog

알고리즘과 SDK 사이에서 사용할 표준 행동 이름이다.
SDK의 실제 JSON 모션 이름은 Motion Executor 한 곳에서만 변환한다.

| 표준 행동 | 의미 | 현재 알고리즘 명령 | SDK JSON 이름 |
|---|---|---|---|
| WALK_FORWARD | 일반 전진 | STRAIGHT | 미정 |
| WALK_APPROACH | 공 접근 전진 | APPROACH | 미정 |
| WALK_SLOW | 느린 접근 | SLOW_APPROACH | 미정 |
| WALK_FINE | 아주 짧은 전진 | FINE_FORWARD_STEP | 미정 |
| TURN_LEFT | 좌회전 | TURN_LEFT | 미정 |
| TURN_RIGHT | 우회전 | TURN_RIGHT | 미정 |
| ADJUST_LEFT | 좌측 미세 보정 | ALIGN_LEFT | 미정 |
| ADJUST_RIGHT | 우측 미세 보정 | ALIGN_RIGHT | 미정 |
| WALK_BACKWARD | 후진 | RETREAT_GOAL | 미정 |
| PICKUP | 공 줍기 | PICKUP_NOW | 미정 |
| SHOT | 슛 동작 | SHOT | 미정 |
| HURDLE_APPROACH | 허들 접근 | APPROACH_HURDLE | 미정 |
| HURDLE_CROSS | 허들 통과 | GO | 미정 |
| HEAD_LEFT | 목 왼쪽 | HEAD_SCAN_LEFT | 미정 |
| HEAD_RIGHT | 목 오른쪽 | HEAD_SCAN_RIGHT | 미정 |
| HEAD_CENTER | 목 중앙 | HEAD_CENTER | 미정 |
| RECOVERY_LEFT | 좌측 복구 | RECOVER_LEFT | 미정 |
| RECOVERY_RIGHT | 우측 복구 | RECOVER_RIGHT | 미정 |
| STOP | 안전 정지 | STOP | 정책 미정 |
| CROSS_FINISH | 결승선 통과 | CROSS_FINISH | 미정 |

## 원칙

- 알고리즘은 SDK의 한글 JSON 이름을 직접 사용하지 않는다.
- JSON 이름 변환은 Motion Executor 한 곳에서만 한다.
- STOP은 일반 모션 이름이 아니라 별도 안전 정지 API가 될 수 있다.
- 하나의 행동 요청에는 완료 또는 실패 상태가 정확히 한 번 반환되어야 한다.
