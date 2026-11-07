# Pinky 주행 시험 — 관제 구현 및 MQTT 이식 가이드

이 문서는 **독립 CLI 주행 시험 버전**의 연결 계약이다. 운영 통신 규약 전체를 대체하지 않는다.
공통 envelope 1.0 위에 profile=roport-pinky-driving-trial-v1을 명시한다.
Pinky ROS 주행·Nav2 연결 및 독립 정지 감시는 [PR #35](https://github.com/kernel-robot-8th/kernel-robot-repo-1/pull/35)의 드라이버와 함께 사용한다. 실장비 방향·Nav2·정지 시간은 현장 검증이 필요하다.

## 먼저 선택: 1대 / 2대, 가상 / 실물

**같은 수신 모듈·메시지 스키마를 사용하고 시나리오 설정만 바꾼다.**
한 번 생성한 `trial.json`을 관제와 해당 Pinky PC 모두에 그대로 배포한다.
각 PC에서 따로 생성하면 binding/site/control이 달라 연결되지 않는다.

| 구분 | 1대 — `single_pinky` | 2대 — `two_pinky` |
| --- | --- | --- |
| 기체/역할 | pinky1 / logistics 하나. N11에서 같은 기체를 스마트카트로 가정 | pinky1 / logistics + pinky3 / smartcart |
| 흐름 | N16 → N18 → N19 → N11 → 더미 2회 → N12 → Nav2 골 | 창고 상차 → N11 입고 → 창고 복귀 → 물품 요청 → 스마트카트 N11 → N12 → Nav2 골 |
| 관제 이동·전환 명령 수 | 4개 | 7개 |
| 수동 더미 승인 | 입고·카트 적재 2회 | 창고 상차·입고·카트 적재 3회 |
| 물품 요청 입력 | 생략 | 창고 복귀 후 `request trial-package-1` |

`scenarios/single|two/simulation|hardware/`에 네 설정을 분리한다.
가상 버전은 같은 PC의 loopback 브로커만 허용한다. 실물 버전은 ROS hook을 연결한 핑키와 공통 LAN 브로커를 사용한다.
각 설정은 서로 다른 site/control/binding을 사용한다. 한 실물 로봇에는 한 시나리오만 연결한다.
시나리오 전환은 기존 프로세스 종료·정지 확인 → 새 설정/역할 배포 → 별도 이력으로 시작 순서다.

`Interface Specification (comm)`은 운영 규약 검토용이며 이 시험에 그대로 재생할 수 없다.
이번 연동은 **이 가이드 + 같은 ZIP의 `message.schema.json` + `trial.json`**을 한 묶음으로 따른다.
기존 페이지의 `rotate_180`, 부가 route 필드, 별도 depart/dock, goal_id만 있는 navigate를 혼합하지 않는다.
메시지 규격은 trial-v1 그대로이며 배포 키트 revision은 `2026-11-05-reviewable-handoff`다.

## 1대 흐름: 입고부터 공용공간까지

설정 생성기의 `--single-robot` 옵션을 사용한다(`scenario=single_pinky`).
`robots.logistics`의 한 기체·binding만 등록하며 같은 기체를 스마트카트로 가정한다.
MQTT 토픽·메시지 스키마·Pinky 이식 모듈은 기존과 같다. 팀원 코드 수정은 포함하지 않는다.

| 순서 | 관제 동작 | 실제 조건 / 시험 가정 |
| --- | --- | --- |
| 1 | `route.requested / to_delivery` | N16 → N14 → N02 → N10 → **N18** → N07 → N23 → **N19 → N11** |
| 2 | `WAIT_INBOUND`에서 입고 완료 더미 승인 | N11 도착·정지·라인 해제 후만 허용 |
| 3 | `WAIT_CART_LOAD`에서 적재 완료 더미 승인 | 같은 기체가 N11에 스마트카트로 들어와 있다고 가정. 실제 복귀/재진입 명령 없음 |
| 4 | `route.requested / cart_to_switch` | **N11** → N19 → N24 → N09 → N08 → **N12** |
| 5 | `mode_switch.requested`, 완료 후 `navigate.requested` | N12 정지 → Nav2 구동권·위치 추정 준비 확인 → 공용공간 골 전송 |

두 더미는 관제 CLI의 `dummy <gate.stop_session_id>` 입력이다. 실제 OMX 명령은 발행하지 않는다.
첫 더미는 입고 이력을 저장하고 새로운 승인 토큰으로 `WAIT_CART_LOAD`를 만든다.
두 번째 gate의 `arrival_execution_id`는 원래 N11 도착 실행을 참조하고, `assumed_role=smartcart`로 가정을 기록한다.
첫 토큰을 다시 입력해도 적재 승인이나 N12 출발이 되지 않는다. 두 번째 승인도 N11 정지 상태를 재확인한다.
별도 `request` 입력은 필요하지 않다. 두 번째 핑키의 상태 응답을 기다리지 않는다.

공용공간 골 후보는 v13 실측 좌표 **(410, 190) cm**이며, layout의 frame 변환 후
`map` 좌표 **x=2.576678 m, y=-0.115347 m, yaw=0.001745 rad**를 사용한다.
지도에서 골 주변 ±15 cm가 비점유 셀인지 검사했다. Nav2 경로 생성·inflation·실제 장애물 안전성을 확인한 값은 아니다.
1대·2대 실기 템플릿 모두 이 골 후보가 들어가며 `field_verified=false`다. 1대의 N16 출발은 팀원 확인값(-1.5690 rad, none, 0.03 m/s), N11 출발은 기존 loading 자세에서 계산한 동쪽 후보(frame yaw, none, 0.03 m/s)를 넣는다. 2대 템플릿의 출발 방향/동작은 현장 입력이 필요하다.
특히 N11에서 N19로 나올 때 필요한 후진/회전과 실제 골 접근 경로는 현장에서 확인해야 한다.

기존 2대 시나리오는 옵션 없이 유지한다. 설정과 시험 이력은 시나리오별 새 파일로 분리한다.

## 1. 이번 구현의 경계

| 구분 | 구현/담당 |
| --- | --- |
| 관제 | 노드 경로 발행, 실제 완료 응답 검증, 더미 적재 입력, 요청 물품 1개, 단계 전환, 오류 시 HOLD |
| 이식 모듈 | Paho MQTT 5, 스키마/대상/유효기간 검증, 중복 방지, 파일 이력, 중요 이벤트 재전송, 저장 ACK |
| Pinky 팀원 | 노드 인식·회전·후진·정지, 최신 관측 제공, 구동권 전환, Nav2 action 결과 연결 |
| 이번에 제외 | 실제 OMX·영상/AI·WSS·QR·카트 복귀·다중 동시 작업·웹 시험 버튼·운영 DB 선정 |

관제 입력은 CLI다. 웹/운영 DB와 합쳐진 운영 실행부는 아니다.
로컬 JSON 시험 이력은 운영 DB의 대안으로 확정한 것이 아니며 API/UI가 이 파일을 읽지 않는다.
이번 시험은 별도 site_id와 설정에 명시한 기체 binding으로 격리한다.
같은 로봇에 기존 실행부와 시험 실행부를 동시에 연결하지 않는다.

## 2. 2대 시나리오 흐름

관제 CLI → Mosquitto/EMQX → Pinky MQTT 모듈 → 팀원 ROS 실행부
Pinky 측 측정/결과 → MQTT event → 관제 검증·파일 저장 → 다음 명령 또는 HOLD

| 순서 | 관제 명령/입력 | Pinky가 해야 할 일 | 다음 단계 조건 |
| --- | --- | --- | --- |
| 1 | route.requested / to_loading | 출발점 → 창고 상차점 라인 주행 | 목표 노드에서 정지 + 라인 해제 |
| 2 | 운영자 dummy / WAIT_LOAD | 정지 유지 | 현재 정차 세션과 일치하는 더미 적재 |
| 3 | route.requested / to_delivery | 창고 → 인도장 라인 주행 | 목표 노드 정지 |
| 4 | 운영자 dummy / WAIT_INBOUND | 정지 유지 | 더미 선반 입고 |
| 5 | route.requested / return_warehouse | 물류 Pinky 창고 복귀 | 복귀 완료 후 WAIT_REQUEST |
| 6 | request / 물품 목록 | 스마트카트 대기 | 보관한 물품 1개·수량 1만 허용 |
| 7 | route.requested / cart_to_delivery | 스마트카트 → 인도장 | 목표 노드 정지 |
| 8 | 운영자 dummy / WAIT_CART_LOAD | 정지 유지 | 더미 카트 적재 |
| 9 | route.requested / cart_to_switch | 인도장 → N12 | N12 정지 + 라인 해제 |
| 10 | mode_switch.requested | 라인 중지·구동권 Nav2 이전·위치 추정 확인 | mode_switch.completed의 모든 준비 조건 |
| 11 | navigate.requested | 전달된 map 좌표로 Nav2 주행 | SUCCEEDED + 정지 + 목표 허용오차 충족 |

이번 시험에서 더미 입력은 OMX MQTT를 사칭하지 않는다. 관제에 simulated=true로 기록한다.
더미는 Pinky의 도착/정지를 대신할 수 없다. 정차 세션은 마지막 route의 execution_id이며 task·step·로봇·노드·물품·수량과 함께 기록된다.
물류 Pinky가 창고에 복귀한 뒤에만 스마트카트를 출발시켜 인도장 중복 진입을 피한다.

v14 후보 경로(2026-11-06 지도, 02 인도장 분기 N23·N24 추가)는 설정 파일에서 수정할 수 있다. P4/P7의 운영 위치를 확정하는 변경은 아니다.

| 단계 | 후보 노드 배열 |
| --- | --- |
| to_loading | N16 → N14 → N02 → N10 → N18 → N01 |
| to_delivery | N01 → N18 → N07 → N23 → N19 → N11 |
| return_warehouse | N11 → N19 → N23 → N07 → N05 → N15 → N14 → N16 |
| cart_to_delivery | H1 → N09 → N24 → N19 → N11 |
| cart_to_switch | N11 → N19 → N24 → N09 → N08 → N12 |

설정 생성기는 웹 지도와 동일한 maps/airport_map.yaml 및 layout/roport_airport.yaml을 읽어 map_version·graph_version과 인접 연결을 확인한다.
인접 연결 검사는 실제 회전 가능성·장애물·후진 거리·AMCL 정합성을 증명하지 않는다.
실기 템플릿은 출발 방향·출발 동작이 미지정이고 골은 후보이므로 현장 확인 없이 실행되지 않는다.

## 3. 연결할 때 Pinky IP가 필요한가

**MQTT 명령 전송에는 Pinky IP가 필요하지 않다. 관제와 Pinky 모두 같은 브로커 주소로 접속한다.**
대상 선택은 site_id + robot_id + binding_id로 한다. client_id는 접속 세션 식별자이며 robot_id와 역할이 다르다.

| 정보 | 넣는 곳 | 의미 |
| --- | --- | --- |
| 브로커 IP/DNS | 양쪽 ROPORT_MQTT_HOST | Mosquitto가 실행 중인 PC 또는 EMQX 접속 주소 |
| 포트 | ROPORT_MQTT_PORT | 기본 1883, TLS 리스너 예시 8883. 실제 브로커 설정과 일치 |
| TLS / CA | ROPORT_MQTT_TLS=1, ROPORT_MQTT_CA_FILE | CA는 사설 CA 사용 시 경로, 미설정 시 시스템 신뢰 저장소 |
| 계정/암호 | ROPORT_MQTT_USERNAME / PASSWORD | 코드·Git·문서에 실제 값 저장 금지. 현재 모듈은 암호 사용 시 TLS 필수 |
| site_id | 관제 JSON / Pinky 생성자 site | 시험 전용 동일 값. 예: pinky-trial |
| control_id | 관제 JSON / Pinky 생성자 control | 명령 소유자. 예: control-trial |
| robot_id | 관제 robots / Pinky 생성자 robot | 고정 슬롯 pinky1, pinky2, pinky3 중 하나 |
| binding_id | 관제 robots / Pinky 생성자 binding_id | 해당 시험 기체 연결 식별값. 생성된 값을 양쪽에 동일하게 복사; 비밀키 아님 |
| client_id | MqttConfig / mqtt_config 인수 | 관제·각 Pinky마다 중복 금지 |
| boot_id | 모듈 자동 생성 | 프로세스 시작마다 바뀜. 설정 파일에 고정하지 않음 |
| journal_path | 각 프로세스 실행 인수 | 재시작 후에도 유지되는 로컬 파일. /tmp 사용 금지 |
| 지도·그래프·목표 버전 | 관제 JSON / Pinky observe | 같은 원본/좌표계인지 확인 |
| Pinky IP | SSH·진단 도구 | 모듈 설치/원격 로그인용으로만 별도 사용 |

Pinky에서 localhost를 쓰면 Pinky 자신을 가리킨다. 브로커가 관제 PC라면 그 PC의 LAN 주소/DNS를 넣는다.
같은 Wi-Fi여도 AP client isolation, 방화벽, 브로커가 loopback에만 listen하는 설정이면 연결되지 않는다.
TLS 인증서 이름과 접속 host가 맞아야 하며 검증을 끄는 옵션은 제공하지 않는다.
EMQX 전환은 host·port·TLS·CA·계정 변경으로 가능하도록 구성했다. 실제 EMQX 연결 시험은 별도다.
브로커 ACL로 각 Pinky의 자기 cmd/ack 읽기와 자기 event 쓰기, 관제의 반대 권한을 제한한다. payload source 검사만으로 인증되는 것은 아니다.

## 4. 토픽·메시지 계약

모든 아래 메시지는 MQTT 5 / JSON UTF-8 / QoS 1 / retain=false다. cmd는 10초 Message Expiry 및 수신 expires_at 검사도 적용한다.

| 방향 | 토픽 | 메시지 |
| --- | --- | --- |
| 관제 → Pinky | roport/v1/{site}/robot/{pinky}/cmd | status.requested, route.requested, mode_switch.requested, navigate.requested |
| Pinky → 관제 | roport/v1/{site}/robot/{pinky}/event | status.completed, 각 작업의 accepted/completed/failed/rejected, route.progress |
| 관제 → Pinky | roport/v1/{site}/robot/{pinky}/ack | event.ack |

공통 루트 필수: schema_version, profile, message_id, type, site_id, source, target, sent_at, task_id, step_id, command_id, execution_id, payload.
명령만 expires_at을 추가한다. 모든 시간은 UTC offset이 있는 ISO 8601이다.
binding_id/boot_id/phase_revision/observed_at 등 작업 데이터는 payload 안에 둔다. 임의 필드는 거부한다.
첨부 message.schema.json은 전체 JSON Schema이며 trial_wire.validate_trial이 실제 수신 검사다.
examples.json은 형식 예시다. 과거 시각·예제 binding·예제 좌표를 실기에 재생하지 않는다.

| 명령 | payload / 수신 측 사용처 | 반환 |
| --- | --- | --- |
| status.requested | binding_id. 동작을 시작하지 않고 실제 최신 상태 조회 | status.completed |
| route.requested | binding_id, expected_boot_id, graph_version, start_node, start_heading_rad, start_maneuver, route, goal, speed_limit_mps | route.accepted → route.progress → route.completed 또는 failed |
| mode_switch.requested | binding_id, expected_boot_id, node_id=N12, from_mode=line, to_mode=nav2, map_id/map_version | accepted → completed 또는 failed |
| navigate.requested | binding_id, expected_boot_id, goal_id/goal_version, map_id/map_version, frame_id=map, pose{x,y,yaw}, position_tolerance_m/yaw_tolerance_rad | accepted → completed 또는 failed |
| event.ack | acked_message_id, status=stored | 모듈이 해당 outbox 항목 제거; 로봇 동작 없음 |

route 배열은 출발 노드를 포함한다. goal={node_id, location_id:null 또는 ID, on_arrival:"hold"}.
start_heading_rad는 map 기준 rad이며 명령 수신 시 관측 방향과 0.35 rad 이내를 요구한다. N11에서는 load_done 회전 전 상차 자세를 넣으며, 다음 구간의 주행 방향을 넣지 않는다.
start_maneuver는 none / turn_back / back_out이다. 후진 거리·회전 완료 판정은 Pinky의 현장 설정으로 정의하고 미지원 동작은 validate_action에서 거절한다.
속도는 m/s, 현재 입력 범위 0.01~0.2, 예시 0.1이다. 이 수치를 현장 안전 속도 검증 완료로 해석하지 않는다.
별도 depart.permitted/dock.requested는 이번 실행부에서 발행하지 않는다. 현재 정차의 더미 승인 후 다음 route가 출발 허가를 함께 가진다. 실제 도킹 정밀도나 OMX 안전 확인을 표현하는 규약은 후속 통합 대상이다.

## 5. Pinky가 반환할 정보와 완료 조건

모든 응답 payload의 공통 정보:

| 필드 | 의미 / 규칙 |
| --- | --- |
| binding_id, boot_id | 현재 연결 기체와 프로세스. 모듈에서 생성/추가 |
| observed_at | 센서·실행부 상태를 실제 관측한 시각. 캐시를 읽은 현재 시각으로 덮어쓰지 않음 |
| phase_revision | 해당 명령 응답 순번 1부터 증가. 모듈에서 관리 |
| node_id | 실제 확인된 현재 노드. Nav2 이동 등 모르면 null |
| mode | line / nav2 / unknown |
| stopped, line_enabled | 실제 정지 여부, 라인 출력 활성 여부 |
| nav2_ready, localization_ready | Nav2 준비, AMCL·TF 등 위치 추정 사용 가능 여부 |
| drive_owner | none / line / nav2. 동시에 두 구동원이 활성화되면 안 됨 |
| graph_version, map_id, map_version | 현재 Pinky에 적용된 버전 |
| heading_rad | map 기준 현재 방향 rad |
| fault | 정상 null, 오류 시 식별 코드 |

route.progress/completed에는 seq, total, prev_node, next_node를 추가한다.
seq는 배열의 0-based 인덱스이고 total은 출발 노드를 포함한 배열 길이다.
seq와 node_id·앞뒤 노드는 전달받은 route와 정확히 일치해야 한다.
완료는 마지막 인덱스 + 실제 정지 + line_enabled=false + mode=line일 때만 보낸다.
accepted는 모듈이 검증·이력 저장 후 자동 발행한다. MQTT 수신이나 PUBACK은 완료가 아니다.

mode_switch.completed는 N12·정지·line_enabled=false·mode=nav2·drive_owner=nav2·nav2_ready=true·localization_ready=true 및 지도 일치가 모두 필요하다.
navigate.completed는 goal_id/goal_version/frame_id, 실제 pose{x,y,yaw}, nav2_result="SUCCEEDED"를 추가한다.
관제는 실제 pose의 거리/각도 오차를 명령의 허용오차와 비교한다. 명령 목표 좌표를 결과 pose로 그대로 복사하면 안 된다.

failed/rejected는 error_code를 추가한다. 예: line_lost, obstacle_timeout, nav2_failed, mode_switch_failed, unsupported_maneuver.
브로커 연결·JSON 파싱·binding 등이 잘못되어 신뢰할 수 없는 요청은 응답 없이 거부할 수 있다.
상태 조회는 1초 주기, 관측/수신 최대 나이 3초, 미래 시각 허용 0.5초이므로 PC·Pinky 시각 동기화가 필요하다.
새 명령과 이벤트의 sent_at도 3초 이내여야 한다. 명령의 10초 expires_at과 별도로 이 더 짧은 신선도 검사를 적용한다.
이는 시험 값이다. telemetry/motion의 기존 개발 프로파일(roport-state-dev-v1 / schema 1.1)은 별도이며 이번 status 응답을 대신하지 않는다.

## 6. 핑키에 모듈 이식

1. 첨부 키트를 풀고 pinky_mqtt 폴더 전체를 팀원 Python 패키지 안에 복사한다. 파일별 import를 수동으로 바꾸지 않는다.
2. 실제 Pinky 환경에서 Python 3.12와 paho-mqtt==2.1.0을 준비한다. MQTT 폴더 자체는 ROS·FastAPI·Qt·DB 드라이버를 import하지 않는다.
3. 기체별 연결 값과 관제 JSON의 binding을 맞춘다. broker 계정은 환경변수나 기체 비밀 저장소로 공급한다.
4. 아래 4개 hook과 비동기 결과 큐를 팀원 실행부에 연결한다. 모듈 검증과 ROS 구동 로직은 분리한다.
5. 처음에는 모터를 비활성화한 hook으로 스키마/콜백/반환을 검사한 뒤 현장 안전 조건하에 구간별 주행 시험을 한다.

| hook | 팀원 구현 |
| --- | --- |
| observe() | 5절 상태 dict + 실제 observed_at 반환. stale·unknown을 준비 완료로 바꾸지 않음 |
| validate_action(command) | 구동 전 전체 노드/엣지·지원 출발 동작·센서/비상정지·독점 구동권 확인. 실패 시 ValueError("식별_코드") |
| dispatch(command) | route/mode_switch/navigate를 해당 ROS 동작으로 전달하고 빠르게 반환. 작업 완료까지 블록하지 않음 |
| safe_stop(reason) | 라인 출력 중단, Nav2 취소, 속도 0, 로컬 구동원 차단. 세부 방식은 팀원 드라이버에 맞게 구현 |

새 runner는 `heartbeat()`가 있는 드라이버에 MQTT poll 이후 lease 갱신을 요청하고 종료 시 `close()`를 호출한다. 실제 Pinky trial_driver는 이 heartbeat와 별도 trial_guard를 요구한다. 구형 키트와 새 드라이버를 혼용하지 않는다. 자세한 ROS 배포는 `services/roport-pinky/src/roport_pinky/README.md`를 따른다.

공통 실행기는 `pinky_mqtt.trial_runner`다. `--config`와 `--role`에서 식별값을 읽어
PinkyTrialClient를 생성하며, 1대/2대에 따라 팀원 수신 코드를 복제할 필요가 없다.
기본 드라이버는 PR #35의 `roport_pinky.trial_driver:create_driver`다. 별도 드라이버로 교체할 때는 아래 factory 계약을 따른다.

~~~python
# my_pinky/trial_bridge.py — 예시 이름
from my_pinky.ros_trial_adapter import TrialDriver  # 팀원 구현


def create_driver(profile, role):
    return TrialDriver(profile=profile, role=role)
~~~

반환 객체는 위 4개 hook과 `drain_results()`를 제공해야 한다.
`dispatch`는 step_id나 logistics/smartcart라는 이름에 동작을 하드코딩하지 않고,
`command["type"]`과 `payload.route` / `to_mode` / `pose`를 실제 동작에 연결한다.
1대 시험에서는 logistics 역할의 동일 Pinky가 Nav2까지 처리한다.
모듈 import나 factory 생성 시에는 구동을 시작하지 않는다. 명령 검증 후 dispatch에서만 시작한다.
ROS spin은 별도 스레드/실행기에 두고 아래 결과 큐는 빠르게 반환해야 한다.

~~~python
# drain_results()가 반환할 한 항목의 형태. 값은 실제 주행에서 얻는다.
(
    original_command,
    "progress",
    {"seq": 2, "total": 4, "prev_node": "N09", "next_node": "N11"},
)
~~~

driver.drain_results()는 팀원이 만든 비블로킹 결과 큐다.
ROS callback은 원본 command를 보존해 큐에 넣고 MQTT 소유 스레드 한 곳에서 report한다.
지난 명령의 늦은 callback은 command_id/execution_id가 달라 무시된다.
route 결과 details 예: seq=2, total=4, prev_node="N09", next_node="N11".
완료·실패 때도 원본 command를 넘긴다. 중복 요청은 dispatch를 재호출하지 않고 저장 결과를 다시 보낸다.

표준 JSON Schema 검증기를 따로 붙일 필요는 없다. 같은 폴더의 trial_wire.validate_trial을 사용한다.
독자 MQTT 구현을 쓰면 첨부 schema뿐 아니라 이력·boot·경로 의미 검사·저장 ACK·정지 조건까지 동일하게 구현해야 한다.

## 7. 시나리오별 실행

관제 명령은 저장소 `services/roport-control-server` 기준이다. 처음 설치할 때 `npm ci --ignore-scripts && npm run prepare:assets && uv sync --locked`를 실행한다.
Pinky 명령은 ZIP을 푼 위치 기준이며 `python3 -m pip install -r requirements.txt`로 Paho를 준비한다.
시험 이력은 저장소 밖 영속 경로에 두고 프로세스마다 별도 파일을 쓴다. `/tmp`와 Git에는 넣지 않는다.

### 7.1 공통 설정을 한 번 생성·배포

~~~bash
uv run python tools/prepare_pinky_trial.py \
  --map ../roport-pinky/maps/airport_map.yaml \
  --layout ../roport-pinky/src/roport_gz_world/layout/roport_airport.yaml \
  --bundle-dir "$HOME/.local/state/roport/pinky-trial-01/scenarios"
~~~

다음 네 폴더에 각각 `trial.json`, `broker.env.example`, `CONNECT.md`가 생성된다.
이미 있는 bundle 폴더는 덮어쓰지 않는다. 첨부 ZIP에 포함된 설정을 쓰면 재생성하지 않아도 된다.

~~~text
scenarios/
  single/simulation/    1대 가상
  single/hardware/      1대 실물 준비 템플릿
  two/simulation/       2대 가상
  two/hardware/         2대 실물 준비 템플릿
~~~

한 시험에 사용할 **한 폴더**를 선택해 양쪽에 복사한다. 기본 슬롯은 1대 pinky1, 2대 pinky1+pinky3이며
변경이 필요하면 관제 설정 한 곳을 수정한 뒤 동일본을 다시 배포한다.
기체별 config를 따로 편집하거나 시험 도중 binding을 바꾸지 않는다.
`broker.env.example`을 `broker.env`로 복사·수정하고 각 PC에서 `source`한다. 실제 계정은 ZIP에 넣지 않는다.

키트를 다시 만들 때는 생성한 네 폴더를 포함한다:

~~~bash
uv run python tools/build_pinky_mqtt_kit.py \
  --scenarios-dir "$HOME/.local/state/roport/pinky-trial-01/scenarios" \
  --output "$HOME/.local/state/roport/pinky-trial-01/pinky-mqtt-trial-v1-scenarios.zip"
~~~

### 7.2 가상 핑키로 1대/2대 검증

같은 PC에서 `127.0.0.1:18883` 시험용 Mosquitto를 준비한다.
모든 터미널에서 아래 `TRIAL_DIR`을 동일하게 지정한다. 2대는 경로의 `single`을 `two`로 바꾼다.
첨부 ZIP 설정을 사용한다면 그 압축 해제 위치로 경로를 바꾼다.

~~~bash
export TRIAL_DIR="$HOME/.local/state/roport/pinky-trial-01/scenarios/single/simulation"
source "$TRIAL_DIR/broker.env.example"
~~~

모의 Pinky 터미널:

~~~bash
uv run python scripts/mock_pinky_trial.py --role logistics \
  --config "$TRIAL_DIR/trial.json" --journal "$TRIAL_DIR/logistics.json"
~~~

**2대 시험에서만** 다른 터미널에 동일 환경을 설정한 뒤 추가 실행:

~~~bash
uv run python scripts/mock_pinky_trial.py --role smartcart \
  --config "$TRIAL_DIR/trial.json" --journal "$TRIAL_DIR/smartcart.json"
~~~

관제 터미널:

~~~bash
uv run python -m roport_control_server.executor.pinky_trial \
  --config "$TRIAL_DIR/trial.json" --journal "$TRIAL_DIR/control.json"
~~~

| 관제 CLI 순서 | 1대 | 2대 |
| --- | --- | --- |
| 응답 확인·출발 | `status`로 1대 확인 → `start` | `status`로 2대 확인 → `start` |
| 승인 | WAIT_INBOUND → WAIT_CART_LOAD에서 각각 `dummy <현재 gate.stop_session_id>` | WAIT_LOAD → WAIT_INBOUND에서 각각 dummy, 창고 복귀 후 WAIT_REQUEST에서 `request trial-package-1`, WAIT_CART_LOAD에서 dummy |
| 끝 | N12 전환·목표 도착 응답 후 COMPLETE | N12 전환·목표 도착 응답 후 COMPLETE |

토큰은 매 gate마다 새로 출력된 값을 쓴다. 과거 토큰은 재사용하지 않는다.
가상 주행·좌표·Nav2 결과는 합성 데이터다. 웹 시연의 자동 더미/자동 반복은 실물 실행 절차에 포함하지 않는다.

### 7.3 실물 핑키 연결 준비

1. `single/hardware` 또는 `two/hardware`를 선택한다. 모든 route의 `start_heading_rad`와 `start_maneuver`를 현장 값으로 확인한다. 1대 to_delivery는 핑키 담당 확인값 -1.5690 / none / 0.03 m/s가 이미 들어간다. N11 출발 후보와 목표점은 현장 검증이 필요하다. N11 후진/회전, 골·지도 정합성, 구동권 이전과 독립 watchdog을 확인한다.
2. 검증 완료 후에만 `field_verified=true`로 설정한다. 관제와 모든 Pinky에 같은 JSON을 배포하고 공통 LAN 브로커 주소·포트·TLS/계정을 설정한다.
3. PR #35의 ROS 패키지·trial_guard·trial launch를 Pinky README 절차로 준비한 후 아래 실행기로 연결한다. `--check`는 식별·모드·브로커 설정 형태만 검사하고 드라이버 import, 네트워크, 실제 준비 여부는 검사하지 않는다.
4. 관제 CLI로 상태를 확인한 뒤 수동 `start`/`dummy`/`request`를 사용한다. 실물용 모의 핑키나 자동 성공 응답은 실행하지 않는다.

Pinky PC — ROS workspace를 source하고 ZIP 폴더에서 실행한다. ROS Jazzy와 Paho를 모두 사용할 수 있는 Python 환경이 필요하다:

~~~bash
export TRIAL_DIR="$HOME/.local/state/roport/pinky-trial-01/scenarios/single/hardware"
source "$TRIAL_DIR/broker.env"
export ROPORT_PINKY_START_NODE=N16
export ROPORT_PINKY_LAYOUT=/absolute/path/roport_airport.yaml
export ROPORT_PINKY_MAP=/absolute/path/airport_map.yaml
python3 -m pinky_mqtt.trial_runner \
  --config "$TRIAL_DIR/trial.json" --role logistics \
  --journal "$TRIAL_DIR/logistics.json" \
  --driver roport_pinky.trial_driver:create_driver --hardware-authorized
~~~

2대는 양쪽 PC 모두 `two/hardware` 설정을 사용하며 두 번째 PC에서 `--role smartcart`,
`--journal "$TRIAL_DIR/smartcart.json"`과 실제 배치한 `ROPORT_PINKY_START_NODE=H1`로 실행한다. 드라이버 코드는 같다.
관제 PC — 동일 hardware 설정 및 브로커 환경:

~~~bash
uv run python -m roport_control_server.executor.pinky_trial \
  --config "$TRIAL_DIR/trial.json" --journal "$TRIAL_DIR/control.json" \
  --hardware-authorized
~~~

`mode=hardware`, `field_verified=true`, `--hardware-authorized`가 모두 충족되어야 실행된다.
관제는 경로/방향/골의 형식도 검증하지만 물리 안전 여부를 자동 판정하지 않는다.
종료는 `stop`/Ctrl+C다. 관제 종료나 네트워크 단절 시 로봇의 정지는 독립 watchdog과 비상정지로 보장해야 한다.

### 7.4 원본 웹 관제 UI로 실행 · FFT1-152

관제 코드에서 연결하며 통합관제·Pinky 탭은 같은 수신 상태와 조작 패널을 사용한다. Pinky PR #35·새 MQTT 키트·trial-v1 메시지는 그대로 쓴다.
API와 실행부는 같은 Linux PC에서 **Unix 소켓(SOCK_SEQPACKET)**으로 연결한다. 운영 DB를 대신 구현하지 않으며 JSON 이력은 시험 전용이다.

1. 기존 CLI는 정지 상태를 확인한 뒤 종료한다. 같은 시험을 두 실행부로 운영하지 않는다. 동일 journal은 파일 잠금, 동일 브로커/control ID는 로컬 소유권 잠금으로 중복을 거부한다. DNS로 확인한 각 IP와 loopback 별칭(`localhost`/`127.0.0.1`/`::1`)도 함께 잠근다. DNS 대상이 바뀌는 운영 환경은 고정 브로커 주소를 사용한다.
2. 7.3의 동일 hardware 설정·브로커 환경·현장 확인을 준비한다. Pinky runner·trial_guard를 먼저 준비한 뒤 관제 실행부를 켠다. 초기 상태에 fault가 있으면 기존대로 HOLD한다. 아래는 각 터미널에서 `services/roport-control-server` 기준이다.

관제 실행부 터미널(기존 CLI 명령에 `--socket` 추가):

~~~bash
export TRIAL_DIR="$HOME/roport-trial/scenarios/single/hardware"
# 이 터미널에도 7.3의 ROPORT_MQTT_HOST/PORT/TLS 설정을 적용한다.
uv run python -m roport_control_server.executor.pinky_trial \
  --config "$TRIAL_DIR/trial.json" --journal "$TRIAL_DIR/control_$(date +%Y%m%d_%H%M%S).json" \
  --socket "$TRIAL_DIR/trial.sock" --scope line-only --hardware-authorized
~~~

API 터미널:

~~~bash
export ROPORT_MAP="$(realpath ../roport-pinky/maps/airport_map.yaml)"
export ROPORT_MAP_LAYOUT="$(realpath ../roport-pinky/src/roport_gz_world/layout/roport_airport.yaml)"
export ROPORT_TRIAL_SOCKET="$HOME/roport-trial/scenarios/single/hardware/trial.sock"
read -rsp '웹 시험 조작 키 (24자 이상): ' ROPORT_TRIAL_TOKEN
export ROPORT_TRIAL_TOKEN
uv run uvicorn roport_control_server.adapters.api.app:app \
  --host 127.0.0.1 --port 8000 --ws-max-size 4096 --ws-max-queue 4
~~~

`http://127.0.0.1:8000/ui/`를 연다. 시험 소켓이 설정되면 같은 원본 UI에서 자동으로 TRIAL 입력을 선택한다(`?mode=trial`도 호환). 조작 키를 입력한 다음 버튼을 클릭한다. **Enter만 눌러서는 출발하지 않는다.** **1대: 출발 → 입고 더미 승인 → 카트 적재 더미 승인**. **2대: 출발 → 창고 상차 더미 → 인도장 입고 더미 → 창고 복귀 확인 → 시험 물품 요청 → 카트 적재 더미** 순으로 현재 단계에서만 가능하다. 2대는 두 터미널의 디렉터리를 `scenarios/two/hardware`로 맞춘다. 화면에서 시나리오나 노드를 변경하는 기능은 후속이다. `--scope line-only`는 N12 도착·정지 응답을 받으면 완료하며 mode_switch/navigate를 보내지 않는다. Nav2까지 검증할 때는 새 이력 파일로 `--scope nav2`를 명시한다(생략 시 기존 nav2 동작). scope는 관제 이력에 저장하므로 같은 파일의 시험 범위를 변경할 수 없다. 양쪽 trial.json과 MQTT 스키마는 바꾸지 않는다. 토큰을 사용자가 복사할 필요 없이 현재 `stop_session_id`를 보내며, 오래된 탭/중복 클릭은 실행부가 거부한다. 조작 키는 로그인 계정 시스템을 대신하지 않는다. 원격 접속은 인증된 사설망·HTTPS 리버스 프록시 등 배포 보호가 있는 환경에서 구성하며 무인증 공개 인터넷에 노출하지 않는다.

- 지도·OMX 설치 배치·탭은 기존 UI를 유지한다. Pinky는 고정 3자리 중 trial.json에 배정된 기체만 제어하고, 미배정 자리와 OMX 상태는 미수신으로 표시한다. 화면의 등록 편집기로 실기 시험 binding을 변경하지 않는다.
- 화면을 닫거나 API만 재시작해도 실행부는 계속된다. 새로 열면 같은 작업 상태를 받는다. 실행부 재시작은 기존 HOLD 규칙을 유지한다. 기존 journal에 HOLD가 있으면 웹에서도 자동 출발/초기화하지 않는다.
- `시험 중단 (보류)`는 후속 명령·lease 갱신을 막으며 조작 키가 필요하다. 무인증 중단 허용은 팀 권한 정책 확정 전까지 적용하지 않는다. 즉시 물리 정지를 보증하는 비상정지 버튼이 아니다. 실제 정지는 Pinky watchdog·현장 정지 장치로 확인한다.
- 마지막 수신 노드의 실측 좌표와 방향만 표시한다. 이동 중 연속 x/y·바퀴 회전은 만들지 않는다. Nav2 이동 중 좌표는 미수신으로 표시하고 완료 응답의 pose만 표시한다. 화면 지도 해시/노드 그래프가 시험과 다르면 표시·출발·더미 승인을 막는다.
- `mode=demo`는 기존 화면별 모의 재생이고 `mode=trial`은 실제 MQTT 시험이다. 가상 시험도 동일 TRIAL UI를 사용하되 `single/simulation` 또는 `two/simulation`·loopback 브로커·`mock_pinky_trial.py`를 사용한다. 실물용 실행부와 연결하지 않는다.
- 소켓은 사용자 전용 권한으로 만들고 Ctrl+C/SIGTERM 종료 시 제거한다. SIGKILL·전원 차단으로 파일이 남으면 소유 프로세스 종료를 확인한 뒤 소켓 파일만 정리한다. 실행 중 소켓이나 journal을 삭제해 잠금/복구를 우회하지 않는다. Unix 소켓 경로는 107바이트 이하로 사용한다.

실물(`mode=hardware`) 명령은 **접속자와 URL 모두 loopback이거나 서버가 HTTPS로 인식한 요청**만 허용한다. 원격 HTTP는 조회만 가능하며 키 입력/조작을 차단한다. HTTPS 프록시에서는 외부 Host/Origin을 유지하고 Uvicorn이 신뢰할 프록시 주소만 `--forwarded-allow-ips`에 지정한다. 클라이언트가 보낸 `X-Forwarded-Proto`만으로 우회할 수 없다. HTTPS로 접속하기 전에 키를 입력하지 않는다.

레이아웃은 `ROPORT_MAP_LAYOUT`이 우선이며 미설정 시 기존 `ROPORT_TRIAL_LAYOUT`을 사용한다. HTTP/WS 상태 조회는 worker당 최대 200ms 캐시를 공유하고 명령 판단은 실행부에서 새로 조회한다. 실행부가 늦게 연결되어도 상태 수신 후 경로선을 복원한다. 기동 중 잠금·소켓·브로커 준비에 실패한 미출발 READY 이력은 유지하며, 구동이 시작된 이력이나 기존 HOLD는 이 규칙으로 초기화하지 않는다.

HOLD의 최초 오류는 이후 운영자 종료·브로커 종료로 덮어쓰지 않는다. 오류가 있는 status 응답도 관측 정보로 저장하지만 완료·진행 성공 응답의 오류는 계속 거부한다. HOLD에서는 lease 갱신을 재개하지 않는다.

## 8. 오류·재시작·저장 동작

| 상황 | 처리 |
| --- | --- |
| 동일 명령 재수신 | 모든 필드가 같으면 기존 응답 재전송, dispatch 재호출 안 함 |
| 같은 command_id의 내용 변경 | idempotency_conflict로 거부. expires_at을 늘려 과거 명령을 되살리지 않음 |
| 중요 이벤트 ACK 유실 | accepted/progress/completed/failed를 로컬 outbox에 보관하고 1초마다 재전송 |
| 저장 ACK | 관제 파일 fsync 후 event.ack. status.completed는 최신 표본이라 별도 저장 ACK/재전송 대상 아님 |
| 오래된/잘못된 완료·노드·boot·버전 | 다음 명령 생성 금지, HOLD 또는 거부 |
| 응답 없음 | 기본 명령 deadline 300초. 결과 불명으로 HOLD; 자동 새 execution 없음 |
| 연결 끊김/관제 폴링 중단 | 모듈이 감지하면 safe_stop 호출, 활성 중 status 3초 미수신도 정지 요청 |
| Pinky 재시작 | 새 boot. 미완료 이력이 있으면 recovery_required로 후속 실행 금지 |
| 관제 재시작 | 진행 중 이력이 있으면 HOLD. 자동 재개·재발행 안 함 |
| 파일 저장 실패 | 저장 성공 전 구동/저장 ACK 하지 않음 |
| 과거 dummy·요청 중복 | 현재 phase/정차 세션/물품과 다르면 거부 |

JSON 이력은 Linux flock으로 동일 파일의 중복 프로세스를 막고 임시 파일 → fsync → 원자 교체로 저장한다.
별도 경로를 쓰는 다른 관제를 막는 분산 소유권 장치는 아니다. 실기에서는 브로커 ACL과 단일 관제 운영을 함께 보장해야 한다.
파일 상한 4MiB, Pinky 명령 이력 256개이며 자동 삭제하지 않는다. 정지·결과 대조 후 보관하고 다음 시험을 새 이력으로 시작한다.
HOLD를 파일 수정으로 풀거나 미완료 이력을 지워 재시도하지 않는다. 실제 위치·모드·물품·정지 상태를 확인하고 현장 승인 후 새 시험을 준비한다.

Python 프로세스 kill/power loss/블로킹 중에는 safe_stop을 호출할 수 없다.
Pinky 구동 드라이버의 독립 cmd_vel timeout/비상정지 구현과 측정은 필수다. 모듈의 3초 검사를 물리 정지 시간 보장으로 설명하지 않는다.

## 9. 코드 위치·검증·남은 확인

| 파일 | 역할 |
| --- | --- |
| src/roport_control_server/executor/pinky_trial.py | 관제 CLI/웹 공통 상태 전이 |
| executor/trial_socket.py · adapters/api/trial.py · spatial_assets/trial.js | 단일 소유자 IPC · 상태 투영/API · 원본 UI 조작 |
| src/roport_control_server/adapters/mqtt/pinky_trial.py | 이식용 Pinky 수신·반환·중복/정지 경계 |
| adapters/mqtt/trial_runner.py | 동일 설정·역할로 Pinky 모듈과 팀원 driver 연결 |
| adapters/mqtt/trial_wire.py | 19개 메시지 형태, payload 및 의미 검사 |
| adapters/mqtt/journal.py | 시험 파일 저장 |
| tools/prepare_pinky_trial.py | 최신 지도 기반 1대/2대 × 가상/실물 설정 생성 |
| scripts/mock_pinky_trial.py | 모터 없는 Pinky 모의기 |
| tests/test_pinky_trial.py / test_trial_handoff.py | 시나리오·오류·배포 ZIP 별도 프로세스 MQTT 왕복 검증 |

기존 comm와 다른 시험 확장: profile, status 요청/응답, expected_boot_id/binding, start_heading_rad,
mode_switch 명령, 좌표/버전을 포함한 navigate, payload 내부 공통 상태 및 오류.
운영 규약 확정 여부와 별개로 이 구현본과 연동할 때는 첨부 스키마를 정확히 따른다.

팀원 연결 전 확인: 노드/출발 방향/출발 동작, 선택한 시나리오의 robot/binding, map/graph 버전, 시험 목표 pose,
라인/AMCL 상태 제공, 로컬 watchdog, Nav2 구동권 이전, 완료 측정값.
PR #35의 `trial_driver`·`trial_guard`·`trial.launch.py`와 새 이식 키트를 함께 사용한다. heartbeat/close hook이 없는 이전 키트와 혼합하지 않는다. 실제 모터·Nav2 경로와 장애물 회피 검증은 남아 있다.
