# Pinky MQTT v2 연결

관제와 실물·가상 Pinky 모두 `field_wire.py` / `FieldPinkyClient`를 사용한다. 물리 동작은 팀원의 `roport_pinky.trial_driver:TrialDriver`가 맡는다. `trial_wire.py`는 기존 시험 설정·실행부 내부 표현이며 새 MQTT 스키마가 아니다.

토픽은 `roport/robot/{pinky1|pinky2|pinky3}/{cmd|event|ack|motion}`이다. 명령은 cmd, 상태·결과는 event, 저장 확인은 ack, 표시 좌표는 motion이다. motion만 QoS0이고 나머지는 QoS1이며 모두 retain=false다. `message.schema.json`과 `examples.json`은 v2 실제 JSON이며 예시의 ID·시각은 재생용 명령이 아니다.

## 연결 정보

두 PC가 같은 MQTT 브로커로 접속한다. Pinky IP는 MQTT 라우팅에 필요하지 않다. `ROPORT_MQTT_HOST`, `ROPORT_MQTT_PORT`, `ROPORT_MQTT_TLS`를 양쪽에서 맞춘다. 인증정보는 로컬 환경변수로 주입한다. Mosquitto/EMQX 모두 같은 Paho MQTT5 전송 모듈을 쓴다.

설정의 `control_id`, `robots.{role}.robot_id`, `graph_version`, `map_id`, `map_version`은 관제와 일치해야 한다. `site_id`와 `binding_id`는 기존 설정/이력 호환용으로 로컬에 남고 전송 JSON에는 포함하지 않는다. 각 기체의 client ID는 중복되지 않아야 한다.

## 이식

1. 같은 개발 버전의 관제 MQTT 모듈과 Pinky ROS 코드를 사용한다. `tools/build_pinky_mqtt_kit.py`는 신규 `field_wire.py`, `field_pinky.py`, `trial_runner.py`와 의존 모듈·스키마·예시·파일 SHA256을 포함한 자격증명 없는 키트를 만든다. Git 배포도 가능하며 양쪽 커밋/파일 체크섬을 기록한다.
2. Pinky에 `paho-mqtt==2.1.0`을 준비하고 ROS 환경을 source한다. 설정 JSON을 복사하고 초기 기체 위치를 `ROPORT_PINKY_START_NODE`로 지정한다. 이 값은 실제 위치를 자동 측정한 결과가 아니다.
3. 기존 준비된 `line_nav`, Nav2, 독립 drive guard를 사용한다. ROS driver·`line_nav_node`도 이번 버전으로 갱신해야 N12 역전환이 가능하다. 이 문서는 bringup이나 모터를 자동 실행하지 않는다.
4. `python -m pinky_mqtt.trial_runner --config /path/trial.json --role logistics --journal /path/new-journal.json --driver roport_pinky.trial_driver:create_driver --hardware-authorized`로 연결한다. 스마트카트 역할은 `--role smartcart`다. 먼저 같은 인자에 `--check`를 추가하면 장비·브로커에 접속하지 않고 설정만 검사한다. `field_verified=true`는 현장 확인 후 설정한다.

기존 v1 journal을 v2 endpoint에 재사용하지 않는다. 이전 파일은 보관하고 새 파일로 시작한다. 미완료 실행이 있는 v2 journal을 다시 열면 자동 재주행하지 않고 복구 확인을 요구한다. 프로세스가 재시작되면 `boot_version`이 바뀌므로 관제도 새 상태를 확인해야 한다.

## 명령과 반환

| 수신 | 사용처 | 반환/진행 조건 |
| --- | --- | --- |
| `robot_state_request {}` | 최신 준비·위치 조회 | `robot_state`; 동작 watchdog 갱신 없음 |
| `robot_state_request` + `keep_running_command_id` | 현재 명령 유지 확인 | 같은 boot·활성 명령이며 새 메시지일 때만 watchdog 갱신 |
| `line_route_command` | 노드 배열, 시작 방향(rad), 속도(m/s) → 실제 RouteExecutor | `command_status` accepted / in_progress / completed 또는 failed/rejected |
| `set_drive_mode_command` | N12 line ↔ nav2 구동권 전환 | 실제 정지·위치·준비 확인 후 completed |
| `nav_goal_command` | map 좌표(m/rad) → ROS Nav2 action, 선택적 `goal_id`는 내부 목표 이름으로 전달 | action 성공 + 실제 정지 + 목표 오차 충족 후 completed |
| `robot_stop_command` | 지정 활성 명령 정지 | 정지 확인 후 기존 실행 interrupted + 정지 명령 completed |
| `event_stored` | 관제가 결과를 저장했음 | 해당 메시지 outbox 제거 |

`command_status.result`는 각 명령 결과다. Pinky 구현은 당시 node/mode/stopped/구동권·지도·준비 상태를 함께 반환한다. 라인 결과에는 `seq/total/prev_node/next_node`, Nav2 완료에는 `pose/frame_id/navigation_result`가 추가된다. 실패는 `reason`이 필수다. 완료는 MQTT 수신만으로 만들어내지 않는다.

상·하차점에서 출발하려면 `departure.arrival_command_id`가 현재 도착 명령과 같아야 한다. `transfer_command_id`는 관제가 확인한 이송 완료 실행이다. Pinky는 도착 연관성을 검사하고, OMX 완료·팔 복귀·물품 위치·통로 예약 판정은 관제가 담당한다. 기존 수동 trial에서는 더미 승인 이력을 이 필드로 매핑하며 실제 OMX 검증으로 간주하지 않는다.

## N12 복귀

Nav2로 N12 복귀 골에 이동 → 정지 → `nav2 → line` 전환 → 라인 경로 전송 순서다. 복귀 골은 layout N12에서 바디 중심을 동쪽으로 3cm 옮긴 위치, 서쪽 방향에 frame 변환을 적용한 값이다. 로봇 센서 바가 바디 앞 3cm인 현재 설정을 기준으로 한다.

드라이버는 실제 localization 준비, 정지, 목표 오차 5cm/0.1rad 이내, 0.3초 이내 IR 센터 선 검출·오차 0.25 이내를 확인하고 0.5초 연속 확인 후 라인으로 바꾼다. 다음 시작은 N12와 sensor offset=0을 검증한 `confirmed_start`로 전달한다. 실제 IR/위치 정합·캘리브레이션이 맞지 않으면 전환하지 않는다.

## 가상 시험 범위

`VirtualPinky`는 같은 TrialDriver, RouteExecutor, 실측 그래프, wire·중복 방지·watchdog·도착 판단을 쓴다. 센서·모터·Nav2 action만 가상 I/O다. 가상 Nav2는 장애물 없는 이동 모델이며 실제 Nav2 recovery·마커 인식·모터 지연·무선 품질·독립 ROS guard 프로세스 종료는 실물/HIL 검증이 필요하다. 시험 JSON과 코드 체크섬을 보관하고 결과를 실물 완료로 표기하지 않는다.

기존 `run_virtual_trial.py --record`는 기록의 **설정만** 재사용한다. 속도 상한·지도 버전은 대조하지만 당시 custom plan, N12 출발 자세, ROS 노드 상태·기체별 IR/도킹 캘리브레이션·소스 버전을 재생하지 않는다. 원본 plan/scope와 이 제한을 실행 manifest에 남긴다. 새 가상 시험의 기본 속도는 0.08 m/s이며 `--record`는 기록의 속도 상한을 보존한다. 실제 driver는 자기 설정 속도와 명령 상한 중 작은 값을 사용하므로 명령 상한을 실측 속도로 취급하지 않는다.
