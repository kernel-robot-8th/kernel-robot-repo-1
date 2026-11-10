# Pinky 게이트 목표 전송과 Nav2 실패 복구 · FFT1-150

이 문서는 PR64 통합 시험의 `roport-pinky-driving-trial-v1` 계약을 기준으로 한다.
로봇은 경로 계획·우회·제한된 복구를 수행하고, 관제는 목적지·작업 배정·최종 실패 이후의 결정을 소유한다.

## 목표를 보내는 순서

1. `status.requested`로 현재 binding/boot, 노드·정지·구동권·위치 추정 준비를 확인한다.
2. H1에서 출발하면 공통 계획의 `cart_to_delivery`로 N11에 이동하고 적재 확인 후 `cart_to_switch`로 N12까지 라인트레이싱한다. N12 이전에는 게이트 Nav2 골을 보내지 않는다.
3. N12 정지·라인 해제를 확인한 뒤 `mode_switch.requested`를 보낸다.
4. **`mode_switch.completed`를 받은 뒤** `navigate.requested`를 보낸다. `accepted`는 도착이 아니다.
5. 수령 확인 후 복귀는 별도 임무다. 이 변경은 trial-v1에 없는 line 복귀 모드 전환 계약이나 자동 귀환을 추가하지 않는다.

토픽은 `roport/v1/{site_id}/robot/pinky3/cmd`, 응답은 같은 경로의 `/event`다.
기존 `adapters/mqtt/trial_wire.py`의 `command(...)`를 사용하면 envelope·UUID·시간을 생성하고 검증한다.
`source`는 공통 설정의 control_id, `target`은 pinky3이며 task/step/command/execution 식별자를 지정한다.
`binding_id`는 공통 JSON, `expected_boot_id`는 최신 status 응답에서 가져온다. 이전 boot를 재사용하지 않는다.
단위는 **m/rad**, 좌표계는 **map**이다. 도면 cm나 odom 좌표를 그대로 보내지 않는다.

현재 배포된 공통 JSON의 `nav_goal`은 PUBLIC_SPACE_TRIAL 한 곳이며, G1/G2/G3 이름을 좌표로 변환하지 않는다.
네 소스의 `roport_smartcart/config/places.yaml`에는 gate2/marker_id=9가 있으나 실물 pose는 null이다.
따라서 관제가 검증된 좌표를 공통 JSON의 nav_goal 또는 생성 계획의 navigate 단계에 넣어야 한다.
`gate2`라는 문자열만 보내거나 빈 pose를 보내면 안 된다.

## G2 앞끝 약 5 cm 후보

11/9 현장 파일의 벽 평면과 당시 설정된 차체 앞끝·패딩으로 계산한 후보다.
**실제 5 cm 정차를 실측 완료한 값이 아니다.** 적재물 외곽, AMCL 오차, 정차 오차가 달라지면 벽 간격도 바뀐다.
도면의 마커 위치와 로봇 중심 정차 좌표를 구분한다.

`navigate.requested` payload 예시 (`binding_id`/boot/version은 실제 공유 값으로 교체):

```json
{
  "binding_id": "REPLACE_ME",
  "expected_boot_id": "REPLACE_ME",
  "goal_id": "G2",
  "goal_version": "g2-front-5cm-candidate-v1",
  "map_id": "airport_map",
  "map_version": "REPLACE_ME",
  "frame_id": "map",
  "pose": {"x": -0.0227737182, "y": -0.4318848857, "yaw": -3.1398473243},
  "position_tolerance_m": 0.02,
  "yaw_tolerance_rad": 0.10
}
```

새 후보 JSON은 관제에서 검토·현장 확인한 뒤 동일 파일을 참여 PC와 Pinky에 재배포한다.
이전 PUBLIC_SPACE_TRIAL 승인을 새 G2 후보의 field_verified로 재사용하지 않는다.
`trial.launch.py goal_xy_tolerance:=0.02`로 controller의 위치 허용 오차도 맞춘다(기본 0.08 m 유지).
허용 오차 2 cm는 map 추정 오차이며 실제 벽 간격 5 cm 보장이 아니다. footprint/inflation 때문에 경로가 없으면 안전 여유를 무작정 줄이지 말고 정차점·정밀 접근을 다시 검토한다.

## 로봇 복구 정책

- trial Nav2는 1 Hz 경로 재계획을 유지한다.
- 복구 가능한 planner/controller 오류에 한해 대기 3초 → local/global costmap 초기화 → 재계획한다.
- 복구 재시도는 골당 최대 2회, 각 복구 가지 실행은 10초 이내다. 기다림·서비스 지연이 무제한 누적되지 않는다.
- 첫 복구 이후 지도상 이동 5 cm가 확인되지 않는 시간이 30초에 도달하면 `nav2_recovery_timeout`으로 취소·정지한다. 추가 복구 feedback은 이 시간을 초기화하지 않는다.
- 실제 TF 이동 5 cm를 확인하면 그 복구 구간의 시간을 끝내되 누적 복구 횟수는 초기화하지 않는다. 전체 경로 주행 시간을 30초로 제한하는 정책은 아니다.
- 복구 횟수가 2를 초과하는 다른 BT를 사용해도 driver가 `nav2_recovery_limit`으로 중단한다.
- 센서·위치 추정·TF·구동권 이상은 기다리며 주행하지 않고 기존 guard와 driver가 정지시킨다.
- 현장 여유가 검증되지 않은 회전·후진 복구는 포함하지 않는다. 정적 장애물은 정상 경로 재계획으로 우회하며, 대기는 무한 반복하지 않는다.
- 취소 뒤 늦은 성공 응답이 와도 최종 실패를 성공으로 덮어쓰지 않는다.

## 관제 상태 표시와 최종 실패

`navigation`은 status 및 기존 action 응답 payload의 **선택 필드**다. 기존 필수 필드·완료 판정·ACK는 바뀌지 않는다.
진단 phase: navigating / recovering / completed / failed.
필드: goal_id, target_pose, current_pose(null 가능), recoveries, max_recoveries,
recovery_elapsed_s, recovery_timeout_s, nav2_error_code, action_status, error_code.
`nav2_error_code`는 Nav2 원본 수치, `error_code`는 driver 분류다. 0은 원인이 수집되지 않았을 수도 있다.

구형 관제는 알 수 없는 필드를 거부하므로 MQTT 진단은 기본 비활성이다.
**관제와 키트를 같은 커밋으로 갱신한 뒤**, Pinky runner 환경에 `export ROPORT_NAV_DIAGNOSTICS=1`을 지정하고 재시작한다.
그 전에도 복구 정책과 기존 `navigate.failed/error_code`는 동작한다.
ROS `trial/localization_state`의 navigation 진단은 MQTT 옵션과 관계없이 제공한다.
관제는 기존 status 폴링 응답으로 복구 중 표시를 갱신한다. 별도 navigate.progress 이벤트는 추가하지 않았다.
진단 phase 하나로 작업을 완료하지 말고 기존 `navigate.completed`와 정지·자세 검증 결과를 따른다.

| 응답/오류 | 관제 처리 |
| --- | --- |
| accepted 또는 navigation.phase=recovering | 같은 골 중복 전송 금지, 진행/복구 중 표시 |
| nav2_goal_failed | Nav2 원본 코드와 센서·경로 기록 확인, HOLD 유지 |
| nav2_recovery_timeout / nav2_recovery_limit | 복구 한도 초과 표시, 정지 확인 후 운영자 결정 |
| nav2_not_ready / nav2_goal_rejected / nav2_action_error | 위치·TF·서버·센서 준비 점검, 무조건 재전송 금지 |
| nav2_goal_not_reached | action 성공이어도 실제 정지/최종 자세가 불충분, 도착 처리 금지 |
| 최종 failed | 작업 HOLD, 자동 H1 복귀 금지 |

기존 실행부는 HOLD를 UI로 해제하지 않으며 driver도 실패 후 halted를 유지한다.
따라서 현재 시험에서 재시도는 현장 정지·위치·원인 확인 후 runner와 관제 owner를 매뉴얼대로 다시 구성하는 명시적 절차다.
같은 command_id 재전송은 기록 응답을 재사용하며 새 시도가 아니다. 새 ID만 바꿔도 halted/HOLD를 우회하지 못한다.
기존 journal을 삭제하거나 binding을 바꿔 실패를 지우지 않는다.
관제 command_timeout_s는 정상 경로 주행과 복구 여유를 포함해야 하며 30초 복구 한도와 별개다.

## 검증 범위

단위 검사·ROS 모의 피어·실물 설치/빌드와 실제 장애물 주행을 구분한다.
확인할 실물 시험: 일시적 차단 해제 후 재개, 우회 가능한 고정 장애물, 완전 차단 최종 실패,
센서/위치 이상 정지, 중복 골 차단, G2 가까운 정차 실측. 이 문서의 후보 값만으로 완료 처리하지 않는다.


## 실물 복귀 중 회전 반복 이후의 보완

실물 기록에서 controller의 collision ahead 및 spin 시간 초과·backup 충돌 예상 실패를 확인했다.
따라서 trial의 upstream RPP 최소 전방 추종 거리 0.30 m를 그대로 쓰지 않고
nominal 0.15 m / min 0.08 m / max 0.20 m, lookahead_time 1.0 s로 좁은 우회 경로를 따라가는 후보를 적용했다.
이는 코너를 안쪽으로 잘라 갈 가능성을 줄이는 설정이며 실물 우회 성공을 입증한 것은 아니다.
충돌 검사는 켠 상태로 유지하고 footprint/padding/inflation은 줄이지 않는다.

당시 실물 trial은 초음파 /us_sensor/range를 발행하지만 소비자가 0개였으며 local/global costmap은 scan만 사용했다.
초음파가 연결된 Gazebo 경로와 동일하게 동작한다고 설명하지 않는다.
종이 인형 앞에서 초음파가 약 0.94 m를 보고했으므로 인형의 위치·각도·감지 범위를 확인해야 한다.
이번 추종 설정 변경은 초음파 costmap/정지 연결을 추가하지 않는다. 센서에 잡히지 않는 장애물까지 회피한다고 보장할 수 없다.
