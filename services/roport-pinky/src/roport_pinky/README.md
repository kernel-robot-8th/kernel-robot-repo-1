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
  layout_file:=/absolute/path/roport_airport.yaml
# IR 노드도 이 launch에서 실행할 경우에만 ir_node:=true
```

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

trial launch는 `PoseProgressChecker`를 사용해 **15초 안에 0.05m 이동 또는 0.15rad 회전**을 진척으로 판정합니다. 0.03m/s 저속·제자리 회전을 허용하면서 완전히 막힌 상태는 실패시킵니다. controller와 velocity smoother의 직진 속도도 0.03m/s로 맞춥니다. 원본 Pinky Nav2 설정 파일은 수정하지 않습니다.

## 검증 범위

`bash ci/test.sh`: ROS 빌드, 라인 시뮬레이터, 별도 ROS domain의 가상 map/TF/AMCL/ActionServer, owner 프로세스 강제 종료 시험. 실제 controller/progress plugin의 회전·10초 이상 저속 주행·막힘 판정도 합성 odom/TF로 검사합니다. 실제 모터·LiDAR·Nav2 경로 계획의 현장 검증은 별도다. 테스트는 loopback으로 격리하며 bringup을 실행하지 않는다.
