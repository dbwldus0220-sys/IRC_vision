# Motion Catalog

알고리즘과 motion backend 사이에서 사용할 표준 행동 이름이다.
`motion_id`는 mock Executor용 문자열이고 실제 STEP Dynamics command는
`MotionCommandBridgeNode`가 별도로 변환한다.

| 표준 행동 | 현재 알고리즘 명령 | mock motion_id | STEP Dynamics command |
|---|---|---|---|
| WALK_FORWARD | STRAIGHT | `forward` | 1 |
| WALK_APPROACH | APPROACH | `forward` | 12 |
| WALK_SLOW | SLOW_APPROACH | `forward_short` | 6 |
| WALK_FINE | FINE_FORWARD_STEP | `forward_short` | 27 |
| TURN_LEFT | TURN_LEFT | `turn_left` | 2 + angle |
| TURN_RIGHT | TURN_RIGHT | `turn_right` | 3 + angle |
| LINE_LEFT | LEFT | `turn_left` | 2 + angle |
| LINE_RIGHT | RIGHT | `turn_right` | 3 + angle |
| ADJUST_LEFT | ALIGN_LEFT | `adjust_left` | 15 |
| ADJUST_RIGHT | ALIGN_RIGHT | `adjust_right` | 16 |
| WALK_BACKWARD | RETREAT_GOAL | `backward` | 5 |
| PICKUP | PICKUP_NOW | `pick_ball` | 9 |
| SHOT | SHOT | `shoot` | 17 sequence entry |
| HURDLE_APPROACH | APPROACH_HURDLE | `forward_short` | 13 |
| HURDLE_CROSS | GO | `forward` | 14 sequence entry |
| HEAD_LEFT | HEAD_SCAN_LEFT | `head_left` | bridge 미매핑 |
| HEAD_RIGHT | HEAD_SCAN_RIGHT | `head_right` | bridge 미매핑 |
| HEAD_CENTER | HEAD_CENTER | `head_center` | bridge 미매핑 |
| FINE_LEFT | FINE_LEFT | 미매핑 | 미매핑 |
| FINE_RIGHT | FINE_RIGHT | 미매핑 | 미매핑 |
| STOP | STOP | 미매핑 | bridge 미매핑; Dynamics 98 정의만 확인 |
| CROSS_FINISH | CROSS_FINISH | `hurdle` | bridge 미매핑 |

## 원칙

- mock motion ID와 실제 Dynamics command를 같은 계약으로 간주하지 않는다.
- 현재 SDK player backend는 placeholder이며 실제 SDK 호출을 하지 않는다.
- STOP은 일반 모션 이름이 아니라 별도 안전 정지 API가 될 수 있다.
- 하나의 행동 요청에는 완료 또는 실패 상태가 정확히 한 번 반환되어야 한다.
