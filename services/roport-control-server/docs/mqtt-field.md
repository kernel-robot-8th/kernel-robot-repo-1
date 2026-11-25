# Pinky MQTT v2 연결

관제와 실물·가상 Pinky 모두 `field_wire.py` / `FieldPinkyClient`를 사용한다. 물리 동작은 팀원의 `roport_pinky.trial_driver:TrialDriver`가 맡는다. `trial_wire.py`는 기존 시험 설정·실행부 내부 표현이며 새 MQTT 스키마가 아니다.

토픽은 `roport/robot/{pinky1|pinky2|pinky3}/{cmd|event|ack|motion|host}`이다. 명령은 cmd, 상태·결과는 event, 저장 확인은 ack, 표시 좌표는 motion, 호스트 지표는 host다. motion·host만 QoS0이고 나머지는 QoS1이며 모두 retain=false다. `message.schema.json`과 `examples.json`은 v2 실제 JSON이며 예시의 ID·시각은 재생용 명령이 아니다.

## 연결 정보

두 PC가 같은 MQTT 브로커로 접속한다. Pinky IP는 MQTT 라우팅에 필요하지 않다. `ROPORT_MQTT_HOST`, `ROPORT_MQTT_PORT`, `ROPORT_MQTT_TLS`를 양쪽에서 맞춘다. 인증정보는 로컬 환경변수로 주입한다. Mosquitto/EMQX 모두 같은 Paho MQTT5 전송 모듈을 쓴다.

설정의 `control_id`, `robots.{role}.robot_id`, `graph_version`, `map_id`, `map_version`은 관제와 일치해야 한다. `site_id`와 `binding_id`는 기존 설정/이력 호환용으로 로컬에 남고 전송 JSON에는 포함하지 않는다. 각 기체의 client ID는 중복되지 않아야 한다.

## 이식

1. 같은 개발 버전의 관제 MQTT 모듈과 Pinky ROS 코드를 사용한다. `tools/build_pinky_mqtt_kit.py`는 신규 `field_wire.py`, `field_pinky.py`, `trial_runner.py`와 의존 모듈·스키마·예시·파일 SHA256을 포함한 자격증명 없는 키트를 만든다. Git 배포도 가능하며 양쪽 커밋/파일 체크섬을 기록한다.
2. Pinky에 `paho-mqtt==2.1.0`을 준비하고 ROS 환경을 source한다. 설정 JSON을 복사하고 초기 기체 위치를 `ROPORT_PINKY_START_NODE`로 지정한다. 이 값은 실제 위치를 자동 측정한 결과가 아니다.
3. 기존 준비된 `line_nav`, Nav2, 독립 drive guard를 사용한다. ROS driver·`line_nav_node`도 이번 버전으로 갱신해야 N12 역전환이 가능하다. 이 문서는 bringup이나 모터를 자동 실행하지 않는다.
4. `python -m pinky_mqtt.trial_runner --config /path/trial.json --role logistics --journal /path/new-journal.json --driver roport_pinky.trial_driver:create_driver --hardware-authorized`로 연결한다. 스마트카트 역할은 `--role smartcart`다. 먼저 같은 인자에 `--check`를 추가하면 장비·브로커에 접속하지 않고 설정만 검사한다. `field_verified=true`는 현장 확인 후 설정한다.

호스트 지표는 기본 꺼짐이다. `--host-metrics-period-s 5`처럼 주기를 명시하면 `roport/robot/{pinky}/host`로 `host_metrics`(CPU 수·loadavg·메모리 kB·CPU 온도 °C)를 QoS0·retain=false로 보낸다. 읽지 못한 값은 null이며 0으로 바꾸지 않는다. 표시 전용이라 발행 실패는 로그만 남기고 정지·HOLD·출발 판정에 쓰지 않는다. 온도는 thermal zone type `cpu-thermal`/`x86_pkg_temp`에서 읽으며, 현장 기체의 zone 이름이 다르면 `--host-metrics-temp-path <밀리°C 값이 든 파일>`로 덮어쓴다(주기와 함께 지정). 관제는 시험 설정 `host_metrics: {"hosts": [...], "stale_after_s": 15}`가 있을 때만 별도 client(`<client_id>-host`)로 구독하고 `GET /api/hosts/metrics`로 보여 준다.

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

Nav2 실패 후 같은 목적지 재시도는 기존 관제의 명시적 현장 대조 절차를 유지한다.
v2 endpoint도 ROS driver의 `validate_navigation_retry`를 읽기 전용으로 확인하며,
종료 결과가 아직 pending이면 `command_status/rejected`의 `reason=recovery_required`를 보낸다.
관제는 실행 전 거절된 시도의 원래 실패를 보존하지만, 그 사이 발생한 다른 HOLD는 덮어쓰지 않는다.
새 command ID만으로 다른 목표·이전 boot·통신 장애·구동 guard 고장을 해제할 수 없다.
v2의 기본 보고는 fault/reason과 별도 robot_pose이며, 자세한 navigation 진단은 현재 ROS에서 확인한다.
기존 `ROPORT_NAV_DIAGNOSTICS` 옵션만으로 v2 state의 필드를 추가하지 않는다.

Nav2로 N12 복귀 골에 이동 → 정지 → `nav2 → line` 전환 → 라인 경로 전송 순서다. 복귀 골은 layout N12에서 바디 중심을 동쪽으로 3cm 옮긴 위치, 서쪽 방향에 frame 변환을 적용한 값이다. 로봇 센서 바가 바디 앞 3cm인 현재 설정을 기준으로 한다.

드라이버는 실제 localization 준비, 정지, 목표 오차 5cm/0.1rad 이내, 0.3초 이내 IR 센터 선 검출·오차 0.25 이내를 확인하고 0.5초 연속 확인 후 라인으로 바꾼다. 다음 시작은 N12와 sensor offset=0을 검증한 `confirmed_start`로 전달한다. 실제 IR/위치 정합·캘리브레이션이 맞지 않으면 전환하지 않는다.

`confirmed_start`에도 `roport_line_nav`의 옆선 켜짐 확인(`side_rise_only`, 기본 켬)이 적용된다. 출발할 때부터 옆 IR이 본선 가장자리를 계속 밟고 그 run이 N08 옆선까지 이어지면 N08을 노드로 보지 않고 `node_missed`로 정지한다(안전 정지, 관제는 HOLD). 조건은 [roport_line_nav README](../../roport-pinky/src/roport_line_nav/README.md) 「옆선 켜짐 확인」 절에 있다. 실로봇에서 N12 출발 때 이 경우가 얼마나 자주 생기는지는 아직 재지 않았다.

## 정지 경로와 남은 제한

순차 TRIAL과 Fleet의 정지 경로는 다르다. 아래 동작은 현재 코드 기준이며, 실제 정지거리 상한은 별도 실측이 필요하다.

- **순차 TRIAL은 watchdog 정지 경로를 사용한다.** 순차 시험 연결부(`field_trial_transport.py`)는 `robot_stop_command`를 보내지 않고, 정지·HOLD 때 `keep_running_command_id` 상태 요청만 멈춘다. Pinky는 마지막 유효 요청을 받은 뒤 3초가 넘은 것을 다음 `poll()`에서 확인하고 `control_link_lost`로 `safe_stop()`을 요청한다. 이 watchdog 대기 시간 동안 0.06 m/s 정속으로 주행한다고 가정한 이동 거리가 약 18 cm다(0.06 m/s × 3 s). 여기에는 폴링 주기, 정지 명령 반영, 실제 감속 시간이 들어 있지 않으므로 정지거리 상한이 아니며, 실제 정지거리 상한은 실측 전이라 미확정이다. 그동안 `trial_guard`는 로봇 안의 lease로 열려 있다.
- **Fleet은 STOP을 전송한다.** `FleetRuntime.publish_stops()`는 HOLD·운영자 정지 시 실행 허가 갱신을 끊고 현재 명령을 대상으로 `robot_stop_command`를 보낸다. 전송·수신 성공만으로 정차를 인정하지 않으며 정지 결과와 최신 기체 상태를 확인한다. 거절·만료·결과 불명은 복구 대상으로 남긴다.
- **Pinky STOP은 일반 명령 지연·이력 한도와 분리한다.** `robot_stop_command`는 `sent_at`의 3초 창과 일반 명령 이력 1024개 한도 때문에 거절하지 않는다. 만료 시각·현재 기동·대상 명령·동일 ID의 본문 일치·전송 권한 검사는 유지한다. 활성 명령을 멈춘 뒤 3초 이내의 실제 정차·라인 비활성 관측으로만 완료한다.
- **이미 완료된 명령의 늦은 STOP은 무해 완료할 수 있다.** 대상이 같은 기동에서 완료됐고 다른 활성 명령이 없으며 최신 정차가 확인된 경우에만 기존 결과를 보존한 채 STOP 완료를 보낸다. 후속 명령이 실행 중이거나 대상/기동이 다르면 `stop_target_mismatch`, 정차 관측이 오래됐으면 `fresh_stop_required`로 거절한다. 순차 TRIAL의 명시적 STOP 연결과 실물 정지거리 검증은 후속이다.
- **host 토픽 ACL.** `--host-metrics-period-s`를 켜면 현장 브로커 ACL에 `roport/robot/{pinky}/host` 발행 권한이 있어야 한다. 없으면 QoS0이라 조용히 버려지고 관제 화면에는 `unknown`·`stale`만 보인다. 정지·HOLD·출발 판정에는 영향이 없다.

## 가상 시험 범위

`VirtualPinky`는 같은 TrialDriver, RouteExecutor, 실측 그래프, wire·중복 방지·watchdog·도착 판단을 쓴다. 센서·모터·Nav2 action만 가상 I/O다. 가상 Nav2는 장애물 없는 이동 모델이며 실제 Nav2 recovery·마커 인식·모터 지연·무선 품질·독립 ROS guard 프로세스 종료는 실물/HIL 검증이 필요하다. 시험 JSON과 코드 체크섬을 보관하고 결과를 실물 완료로 표기하지 않는다.

기존 `run_virtual_trial.py --record`는 기록의 **설정만** 재사용한다. 속도 상한·지도 버전은 대조하지만 당시 custom plan, N12 출발 자세, ROS 노드 상태·기체별 IR/도킹 캘리브레이션·소스 버전을 재생하지 않는다. 원본 plan/scope와 이 제한을 실행 manifest에 남긴다. 새 가상 시험의 기본 속도는 0.08 m/s이며 `--record`는 기록의 속도 상한을 보존한다. 실제 driver는 자기 설정 속도와 명령 상한 중 작은 값을 사용하므로 명령 상한을 실측 속도로 취급하지 않는다.
