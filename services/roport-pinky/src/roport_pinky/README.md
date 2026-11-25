# roport_pinky

관제 `roport-pinky-driving-trial-v1`의 ROS 드라이버. MQTT 스키마·저장 ACK·재전송은 관제 배포 키트가 맡고, 이 패키지는 라인 주행·Nav2·독립 속도 감시를 연결한다. 메시지 스키마 1.0은 바꾸지 않는다.

## 이번 시험의 동작

| 명령 / 단계 | 로봇 동작·완료 기준 |
|---|---|
| `route.requested` N16→N11 | IR 라인 노드 목록 주행. N11에서 기존 도착 회전 후 `loading` 상태와 실제 정지를 확인해 완료 |
| N11 더미 입고·적재 완료 | 관제 내부의 두 승인 단계. 로봇은 다음 `route.requested`가 올 때까지 대기 |
| `route.requested` N11→N12 | 현재 상차 자세 검사 → `load_done` → 진입선 방향 복귀 → 다음 경로. `start_heading_rad`는 회전 **전** 현재 map 방향 |
| `mode_switch.requested` | N12 `handoff`·실제 정지·라인 비활성 확인. 지도 일치, AMCL/BT 활성, 위치 추정 준비 후 Nav2 구동권 예약. 이것만으로 이동하지 않음 |
| `navigate.requested` | 실제 `NavigateToPose` action 전송. SUCCEEDED + 최신 map→base_footprint TF + 목표 오차 + odom 정지를 확인한 뒤 완료 |

1대 시험 `to_delivery`: `start_heading_rad=-1.5690`, `start_maneuver="none"`, `speed_limit_mps=0.03`(m/s). N11 출발 후보는 map 동쪽 `0.001745329` rad다. 2대 시험의 동명 `to_delivery`는 N01에서 출발하므로 이 방향값을 복사하지 않는다. 관제 제한시간은 명령당 300초로 준비한다(기존 120초는 0.03 m/s 시뮬레이션 주행보다 짧음).

## 구동 연결과 감시

```text
MQTT runner ──heartbeat()──> trial/lease (String JSON, 최대 10 Hz)
line_nav ──line_nav/cmd_vel──┐
Nav2 smoother ──trial/nav2_cmd_vel──> trial_guard ──cmd_vel──> motor bringup
                           └─ 현재 명령 소유자 출력만 전달
```

- `trial_guard`는 별도 프로세스다. 1초 동안 owner lease가 없으면 속도 0을 반복하고 재시작 전까지 잠근다. line/Nav2가 계속 속도를 보내도 lease를 대신 갱신하지 못한다. 구동 입력 자체가 0.3초 끊겨도 0을 보낸다.
- lease는 MQTT 소유 루프에서만 갱신한다. ROS spin 스레드에서 갱신하지 않는다. guard boot·runner session·증가 seq·0.5초 이내 송신 시각으로 과거 lease를 배제한다.
- 드라이버는 guard 상태 0.3초, line 상태 1초, odom 0.5초 이내를 요구한다. Nav2는 TF 0.5초, AMCL pose 2초, 활성 lifecycle 응답 2초 이내를 요구한다. AMCL 위치 분산 ≤0.04 m², yaw 분산 ≤0.1 rad²를 출발 기준으로 사용한다. 현장 위치 정합성도 별도로 확인한다.
- 로컬 YAML+이미지 해시를 관제 map_version과 대조하고, 실제 `/map`의 크기·해상도·원점·점유 값을 확인한다. 잘못된 지도나 오래된 위치로 성공을 만들지 않는다.
- `/cmd_vel` 발행자는 `trial_guard` 하나여야 한다. 기존 cmd_vel_guard·teleop·다른 Nav2 launch와 함께 실행하지 않는다. 동일 ROS 영역에서 다른 주행 명령/목표도 보내지 않는다.
- guard 자체/구동 PC/모터 제어기 고장은 이 소프트웨어 감시만으로 보장하지 않는다. 모터 드라이버의 하위 타임아웃과 현장 비상 정지가 필요하다.

## 로봇 배포·실행 (현장 운영자 수행)

1. 이 변경의 `roport_pinky` 패키지와 관제의 **heartbeat/close hook을 포함한 새 MQTT 키트**를 같이 배포한다. 기존 Confluence 첨부 ZIP은 자동 갱신되지 않는다. 구형 runner는 `runner_heartbeat_missing`으로 이동을 거부한다.
2. ROS Jazzy 환경에서 `rosdep install --from-paths src/roport_pinky src/roport_line_nav src/roport_pinky_line src/pinky_pro/pinky_navigation --ignore-src -r -y`로 선언된 의존성을 준비하고 `colcon build --packages-up-to roport_pinky` 후 workspace를 source한다. 기존 line_nav, Pinky bringup, IR 보정, LiDAR/odom/TF 준비가 전제다.
3. 관제와 동일한 `trial.json`·지도 파일을 복사한다. 1대/2대의 site·binding을 섞지 않는다. 현장에서 배치 방향·N11 회전·N12 위치 추정·공용공간 골·비상 정지를 확인하기 전에는 `field_verified=false`를 유지한다.
4. 안전 조건 확인 후 기존 bringup과 센서를 준비하고 아래 trial 전용 launch를 실행한다. 기존 line_nav/Nav2 launch 대신 사용한다. AMCL 초기 위치를 실제 배치와 맞추고 지도·위치 추정 준비를 확인한다.

```bash
ros2 launch roport_pinky trial.launch.py robot:=REPLACE_WITH_ROBOT_NAME \
  map:=/absolute/path/airport_map.yaml \
  layout_file:=/absolute/path/roport_airport.yaml \
  max_angular:=0.15
# IR 노드도 이 launch에서 실행할 경우에만 ir_node:=true
```

`max_angular`는 Nav2 controller·behavior·velocity smoother의 회전 상한(0.15~1.5 rad/s, 기본0.4)이다. 라인은 별도 `line_rotate_speed`로 제자리 회전 속도·상한을 정한다(0.15~0.4 rad/s, 기본0.15). Nav2 회전을 올려도 라인은 따라 빨라지지 않는다. 기본값은 각 기체의 현장 승인값을 대신하지 않는다. `line_speed`는 라인 기본 속도·병진 상한으로 0.02~0.06 m/s, `max_linear`는 Nav2 상한으로 0.02~0.15 m/s다(둘 다 기본0.03). 라인0.06·Nav2 0.12 m/s 시험에는 `line_speed:=0.06 max_linear:=0.12`를 추가하고 관제 경로의 `speed_limit_mps`도0.06으로 맞춘다. 더 낮은 관제 제한은 계속 준수한다. 노드 감속은 공용 라인 실행부 설정을 따르며 이 launch가 구현하지 않는다.

5. 새 키트를 실행한다. `broker.env`의 host는 **관제 브로커 PC** 주소이며 Pinky IP를 명령마다 넣지 않는다. 키트 venv에는 ROS 패키지를 사용할 수 있도록 `--system-site-packages`와 paho-mqtt 2.1.0이 필요하다.

```bash
source /path/to/robot_ws/install/setup.bash
export TRIAL_DIR=/path/to/kit/scenarios/single/hardware
source "$TRIAL_DIR/broker.env"
export ROPORT_PINKY_START_NODE=N16
export ROPORT_PINKY_LAYOUT=/absolute/path/roport_airport.yaml
export ROPORT_PINKY_MAP=/absolute/path/airport_map.yaml
cd /path/to/kit
/path/to/venv/bin/python -m pinky_mqtt.trial_runner \
  --config "$TRIAL_DIR/trial.json" --role logistics \
  --journal /persistent/path/logistics.json \
  --driver roport_pinky.trial_driver:create_driver --hardware-authorized
```

드라이버 factory 생성/상태 조회만으로 주행하지 않는다. `start_maneuver=none`과 중간 정차 없는 경로를 지원한다. 주행 실패·lease 만료 뒤 자동 재개하지 않는다. 현장 정지·위치 확인 후 trial stack과 runner를 다시 시작하고 새 관제 시험으로 복구한다. 기존 journal을 삭제해 복구를 우회하지 않는다.

## 정차·저속 주행의 위치 상태

`trial/localization_state` (`std_msgs/String` JSON)를 드라이버가 **10 Hz**로 발행하며 정차 중에도 유지합니다. 실행부도 같은 상태를 사용합니다.

| 필드 | 의미 |
|---|---|
| `observed_at`, `frame_id` | 상태를 판정한 UTC 시각, 좌표계 `map` |
| `pose` | 최신 TF의 `{x, y, yaw}` (m/rad). 유효성 실패 시 `null` |
| `localization_ready` | scan·odom·TF 최신성, AMCL 공분산, 지도 일치와 AMCL/BT 활성 상태 확인 |
| `nav2_ready` | 지도 일치·AMCL/BT 활성·action server 준비 |
| `stopped` | 최신 odom의 평면 속도 각 성분 ≤ 0.005 m/s, 각속도 ≤ 0.02 rad/s |
| `source_stamps` | 원본 `amcl_pose`, `scan`, `odom`, `map_to_base`, `map_to_odom`, `odom_to_base` ROS 시각. 미수신은 `null` |

`amcl_pose`는 정차/저속에서 조건부 발행되므로 2초 주기 수신을 요구하지 않습니다. 마지막 AMCL 표본은 공분산 확인에만 쓰고, 현재 위치는 TF에서 구합니다. scan 1초, odom·odom/base TF 0.5초 및 lifecycle 응답 2초 이내 조건을 함께 검사합니다. AMCL의 map/odom TF는 scan마다 갱신되며 기본 `transform_tolerance=1.0`의 미래 시각을 허용합니다. 오래된 AMCL 표본의 시각은 원본 그대로 남깁니다. 판정 타이머가 0.3초 넘게 멈춰도 준비 상태를 해제합니다.

이 ROS 상태는 MQTT trial-v1 스키마를 바꾸지 않습니다. MQTT 응답의 `localization_ready`·`stopped`와 Nav2 완료 판정에 반영합니다. 위치 유효성과 실제 정지는 서로 별도 조건입니다.

웹 지도 표시는 선택 hook `TrialDriver.observe_motion()`으로 별도 연결합니다. 최신 MQTT runner는 이를 5 Hz `motion` 표본으로 전송하며, 기존 runner/주행 명령/완료 판정은 바꾸지 않습니다. 유효한 TF를 우선하고 라인 모드에서 localization이 없으면 실측 layout으로 변환한 `pose_est`를 표시용 추정으로 구분합니다. 정지 중에도 관측 시각을 유지하고 무효 위치는 null을 보냅니다. 배포할 MQTT 파일과 별도 토픽/ACL은 [관제 시험 가이드 11절](../../../roport-control-server/docs/pinky-trial.md#11-실시간-위치-표시--db-없이-연결)을 따릅니다.

trial launch는 `PoseProgressChecker`를 사용해 **15초 안에 0.05m 이동 또는 0.15rad 회전**을 진척으로 판정합니다. 0.03m/s 저속·제자리 회전을 허용하면서 완전히 막힌 상태는 실패시킵니다. controller와 velocity smoother의 직진 속도는 `max_linear`로 맞춥니다. 원본 Pinky Nav2 설정 파일은 수정하지 않습니다.

Pi에서 경로 재계산 중 `follow_path` 목표 수락 응답이 기본 20ms를 넘으면 전체 목표가 실패할 수 있어, trial의 `bt_navigator.default_server_timeout`은 500ms로 지정합니다. 지도 경계 가까운 공용공간 시험점의 주변 검색이 지도 밖을 조회하지 않도록 `planner_server.GridBased.tolerance`는 0.1m로 제한합니다. 이 값은 플래너의 대체 도착점 검색 범위이며, 실제 정지·목표 오차·독립 구동 감시의 완료 조건은 유지합니다.

## 고객 카드 추종 `follow.requested` (FFT1-188)

Nav2 모드에서 노트북 `roport_follow`가 보낸 고객 카드 위치를 Nav2가 따라간다. 별도 cmd_vel 추종기를 쓰지 않으므로 Nav2 비용 지도(LiDAR, `params_file`에 넣은 인형 층)와 `trial_guard`가 그대로 적용된다. 관제 MQTT를 거친 전체 추종 흐름의 실물 검증은 후속이다.

| 구분 | 내용 |
|---|---|
| 드라이버 명령 | `follow.requested` payload `{binding_id, expected_boot_id, map_id, map_version, card_id(35~49), standoff_m(기본 0.5, 0.3~1.2 m), max_duration_s(선택, 1~3600 s), lost_after_s(선택, 기본 1 s = SR-22, 1~4 s. 4 s 는 명시했을 때만 쓰는 시연값이며 노트북 `--follow-lost-s`와 같은 값을 준다)}` |
| 상태 보고 | `observe().follow`(선택 필드, 추종 명령이 살아 있는 동안만) = `{active, follow_mode: "marker"[, follow_session_id]}`. `active`는 Nav2가 추종 goal을 수락했고 0.3 s 이내 `trial_guard` 응답의 owner가 `nav2`(막힘 없음)이며 fault가 없을 때만 true다. 접수 직후·구동권 전환 대기·종료 뒤에는 false이고, 고객이 서 있어 속도가 0이어도 true일 수 있다. payload의 `follow_session_id`는 받은 값을 그대로 돌려주고, `follow_mode`가 `marker`가 아니면 `unsupported_operation`으로 거절한다 |
| 출발 조건 | mode=nav2, 정지, 지도 일치, Nav2·위치 준비, 이전 Nav2 goal 종료 확인, 해당 card_id status 2 s 이내·`tracking`·**`seen=true`**(가려진 동안의 옛 위치로는 출발하지 않음), 그 status와 같은 stamp의 target이 1 s 이내이고 TF로 map 변환 가능. 거절 코드: `not_in_nav2_mode`, `drive_busy`, `map_mismatch`, `nav2_not_ready`, `invalid_card_id`, `invalid_standoff`, `invalid_max_duration`, `invalid_lost_after`, `follow_bt_missing`, `follow_status_stale`, `follow_card_mismatch`, `follow_not_tracking`, `follow_target_stale`, `follow_tf_unavailable` |
| 입력 | `follow/target` `geometry_msgs/PointStamped`(frame `base_footprint`, stamp = 로봇 카메라 시각), `follow/status` `std_msgs/String` JSON. 둘 다 reliable depth 5. target은 stamp가 같은 status가 `tracking`·`seen`·요청 card_id일 때만 쓴다 |
| 출력 | `NavigateToPose` goal의 `behavior_tree` = 설치된 `behavior_trees/follow_target.xml`, 이후 `goal_update` `geometry_msgs/PoseStamped`(frame `map`, stamp = target stamp, 최대 4 Hz, BT가 4 Hz로 읽음) |
| 목표 | target을 그 stamp의 `map`←`base_footprint` TF로 옮기고 로봇→고객 직선에서 `standoff_m` 앞. 이미 안쪽이면 제자리에서 고객 쪽을 본다 |
| 구동권 | 기존 navigate와 같이 dispatch 때 lease `nav2`, runner `heartbeat()`가 추종 중 `nav2`를 갱신한다. 종료 즉시 Nav2 취소 + lease `none` → guard가 0 속도. `trial_guard`는 바꾸지 않았다 |

종료(찾기·회전 동작 없음). 모두 `follow.failed`의 `error_code`이며 판정 주기는 0.1 s다.

| 조건 | error_code | 드라이버 잠금 |
|---|---|---|
| status `lost`(노트북 기준 `lost_after_s` 못 봄)·`searching`, 또는 target이 `lost_after_s` + 0.5 s 넘게 끊긴 순간 마지막 status가 `seen=false`(그동안 마지막 목표로 계속 감) | `follow_lost` | 없음, 새 follow로 다시 시작 |
| 해당 card_id status 2 s(11-21 시험값) 넘게 미수신, 또는 target이 끊긴 순간 status도 0.5 s 넘게 없음(노트북 꺼짐·다른 카드) | `follow_status_stale` | 없음 |
| 카드가 보인다는 status는 오는데 받아들인 target이 로봇 시각 1 s 넘게 없음 | `follow_target_stale` | 없음 |
| `max_duration_s` 지남 | `follow_max_duration` | 없음 |
| 새 target을 TF로 못 옮긴 채 1 s | `follow_tf_unavailable` | 있음(재시작 복구) |
| Nav2 action 거부·중단·오류 | `follow_nav2_failed` | 있음 |
| 위치 추정 상실, guard·lease 이상 | 기존 코드(`nav2_not_ready`, `drive_*` 등) | 있음 |
| 관제 추종 일시정지(`robot_stop_command`, 사유 `follow_pause`) | 기존 추종 `interrupted`와 STOP `completed` | 잠금 해제 조작 없이, 결과 이후 최신 정차·유휴 상태가 확인되면 관제에서 재개 |

`follow_target.xml`은 Jazzy `follow_point.xml`의 `GoalUpdater` 구조를 따른다. 고객 점은 고객 자신의 비용 지도 장애물·inflation 안이라 계획이 실패할 수 있어 `TruncatePath` 대신 드라이버가 목표를 standoff만큼 당긴다. 막히면 1 s 기다렸다가 다시 한다(5 cm 이동 없이 30 s 면 드라이버가 멈춤, BT 재시도는 누적 100번까지, 회전·후진 복구와 비용 지도 비우기 없음 — 비우면 카메라로만 보이는 가까운 인형이 지도에서 사라져 부딪힌다). 그 뒤 action이 끝나고 드라이버가 `follow_nav2_failed`로 멈춘다. 추종 goal에는 공용 누적 복구 상한(`MAX_RECOVERIES`)을 쓰지 않는다. 기본 navigate BT·launch 인자는 바뀌지 않는다. 인형 층을 쓰려면 그 층이 들어간 전체 Nav2 설정을 `params_file:=`로 준다. 실로봇(d660) 시험에 쓴 설정은 `config/nav2_follow_real.yaml`(설치 위치 `share/roport_pinky/config/`, 2026-11-21 시험값)이다. 고객이 멈추면 로봇은 standoff에서 `general_goal_checker.xy_goal_tolerance`(trial 기본 0.08 m)만큼 떨어진 곳에 서 있을 수 있다.

관제 MQTT(`roport-control-v2`)의 `follow_target_command`는 [관제 MQTT 명세](../../../roport-control-server/interfaces/mqtt/README.md)에 따른다. Pinky의 `field_pinky.py`와 `field_wire.py`는 관제와 같은 현장 번들에서 배포한다. 관제 경로의 카드 유실 기준은 SR-22 기본 1초, 목표 거리는 0.5 m다. 현장 준비 절차는 [Fleet 배포 가이드](../../../../docs/field/README.md#스마트카트-카드-추종)를 따른다.

## 검증 범위

`bash ci/test.sh`: ROS 빌드, 라인 시뮬레이터, 별도 ROS domain의 가상 map/TF/AMCL/ActionServer, owner 프로세스 강제 종료 시험. 실제 controller/progress plugin의 회전·10초 이상 저속 주행·막힘 판정도 합성 odom/TF로 검사합니다. 실제 모터·LiDAR·Nav2 경로 계획의 현장 검증은 별도다. 테스트는 loopback으로 격리하며 bringup을 실행하지 않는다.

Nav2 제한 복구·관제 실패 처리·G2 후보 전송은 [관제 참고 가이드](../../../../docs/pinky-nav2-recovery.md)를 따른다.

게이트 ArUco 주차는 `ROPORT_GATE_DOCK_CONFIG`로 명시적으로 설정한 경우에만 실행한다.
미설정 시 기존 Nav2 명령을 그대로 수행한다. 카메라 보정/광학 TF/앞면 길이 실측과 관제의
최종 정차 좌표·버전을 먼저 맞춘다. 설정·실패 코드·같은 목적지 재시도는
[관제 시험 가이드](../../../roport-control-server/docs/pinky-trial.md)의 마지막 두 절에 정리했다.
기본 예제의 `field_verified: false`를 사실 확인 없이 바꾸지 않는다.
실측 시에는 앞면–벽 5cm의 최종 위치에서도 마커 전체가 화면 안에 들어오고 연속 검출되는지
확인한다. 마커가 잘리거나 시야를 벗어나면 `gate_dock_marker_lost`로 정지하므로,
검출 유지 시간을 늘려 숨기지 말고 카메라·마커 배치를 보정한 뒤 활성화한다.

마커 없는 정밀 접근(`fine_align.py`, 기본 끔): `ROPORT_FINE_ALIGN_GOALS`에 쉼표로 goal_id를 주면(예: `N12_RETURN`)
그 `navigate.requested`만 Nav2가 목표 20 cm 뒤에 선 뒤 목표 좌표까지 직선 경로를 GateDock 제어기로 보내고
1.5 cm·0.08 rad 안을 확인한다(실패 코드 `fine_align_no_goal`·`fine_align_timeout`(25 s)·`fine_align_outside_envelope`(0.30 m·0.35 rad),
게이트 주차와 같은 `gate_dock_stop_unconfirmed`·`gate_dock_controller_unavailable`·`gate_dock_final_tolerance`). 드라이버의 명령 허용 오차
재확인은 그대로다. `ROPORT_GATE_DOCK_CONFIG`가 같이 있으면 게이트 주차가 우선이고 FineAlign은 켜지지 않는다(경고 로그).
`fine_align_*`는 같은 목적지 재시도 대상(`NAV_RETRY_FAULTS`)이 아니다. `trial.launch.py`는 두 환경 변수 중 하나가 있으면
GateDock 제어기·`gate_dock_goal_checker`(1 cm·0.05 rad)를 넣는다. FineAlign 으로 띄울 때만 GateDock 속도를 최소 접근 0.01 m/s·
감속 최소 0.015 m/s·감속 시작 0.05 m 로 넣는다(2026-11-23 d660 N12 20 cm 접근 19 s). 게이트 마커 주차로 띄우면 변경 전 값
(0.005 m/s·0.005 m/s)을 그대로 쓰고, 감속 시작 거리는 launch 가 넣지 않아 `params_file`에 적은 값(없으면 RPP 기본값 0.6 m)이 쓰인다.
`ROPORT_FINE_ALIGN_GOALS`는 `trial.launch.py`를 띄우는 셸과 드라이버(runner)를 띄우는 셸 **둘 다**에 넣는다. 드라이버에만 있으면
Nav2에 GateDock 제어기·검사기가 없어 마지막 접근이 거절되고 `nav2_goal_failed`로 실패한다(재시도해도 같은 실패).
`params_file`이 GateDock을 이미 가진 파일(`config/nav2_follow_real.yaml`)이면 launch 쪽 환경 변수가 없어도 제어기는 있다.
다만 그 파일의 GateDock 속도는 게이트 주차 값(0.005 m/s·0.005 m/s, 감속 거리 없음)이라, FineAlign 속도는 launch 쪽 환경 변수가 있어야 들어간다(없으면 20 cm 접근이 25 s 를 넘길 수 있다).
19 s 실측은 시험 실행기 `demo_runner --fine-align`(`demo_RET_N12`) 경로 결과다. 드라이버 환경 변수 경로(관제 `N12_RETURN`)는 실로봇 미확인이다.

탑승 시간 알림(선택 hook `set_boarding_alert(on)`, bool 아니면 `invalid_alert`): 켜진 동안 드라이버 노드가
`smartcart/boarding_alert`(std_msgs/Bool, depth 1)에 0.5 s 마다 True, 끌 때 False 한 번을 낸다. `close()`나 runner
가 `heartbeat()`를 1 s 넘게 부르지 않을 때 스스로 끈다(halted·guard 고장으로 구동권이 막혀도 runner 가 부르는 동안은 유지). `safe_stop`은 끄지 않는다(표시는 STOPPED 가 우선). `observe()`에는 넣지 않는다.
관제 번들·fleet·패드 쪽에 필요한 것은 [스마트카트 관제 시나리오 전달 문서](../../docs/smartcart-control-handover.md)에 적었다.

`ROPORT_NAV_DIAGNOSTICS=1`일 때 취소한 Nav2 goal의 종료 결과가 없거나 결과 수신이
예외·취소로 끝나면 `navigation.error_code=nav2_cancel_result_unknown`을 보고한다.
원래 실패 이벤트·fault와 종료 증거는 유지하며 동일 목적지 재시도도 계속 차단한다.
관제는 이를 자동 재시도하지 않고, 작업자가 실제 정지와 Nav2 goal 종료를 확인한 뒤
driver/runner 재기동 및 새 boot·현재 위치·작업 상태를 대조하여 시험을 다시 시작한다.
