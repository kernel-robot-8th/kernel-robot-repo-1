# Pinky 주행 시험 — 관제 구현 및 MQTT 이식 가이드

> **이 문서의 trial-v1 토픽·JSON은 이전 버전 기록이다.** 현재 실제 MQTT 연결은 [v2 연결 가이드](mqtt-field.md)와 [전체 인터페이스](../../../interfaces/mqtt/README.md)를 따른다. 기존 시험 계획/이력은 경계 어댑터 내부에서 유지되며 새 endpoint와 혼합 배포하면 안 된다.

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

공용공간 골 후보는 실측 좌표 **(410, 190) cm**이며(v14에서도 유지), layout의 frame 변환 후
`map` 좌표 **x=2.576678 m, y=-0.115347 m, yaw=0.001745 rad**를 사용한다.
지도에서 골 주변 ±15 cm가 비점유 셀인지 검사했다. Nav2 경로 생성·inflation·실제 장애물 안전성을 확인한 값은 아니다.
1대·2대 실기 템플릿 모두 이 골 후보가 들어가며 `field_verified=false`다. 1대의 N16 출발은 팀원 확인 자세(-1.5690 rad, none), N11 출발은 기존 loading 자세에서 계산한 동쪽 후보(frame yaw, none)를 넣는다. 2대 출발 방향/동작도 후보 값이며 현장에서 확인한다. 생성 설정·자동 계획의 기본 속도 제한은 0.06 m/s이고, 계획에서 선택할 수 있는 범위는 0.015~0.08 m/s다. 이전 관제 실물 시험 속도 0.03 m/s와 증속 검증을 구분한다.

구형 2대 설정의 `speed_limit_mps=0.1`은 현재 계획 검증에서 거부된다. 새 폴더에 설정을 생성하고 기존 지도·골·binding·출발 방향과 대조한 뒤 현장 확인을 거쳐 같은 JSON을 관제와 Pinky에 배포한다. 기존 설정과 journal은 보관하며 자동 덮어쓰기·속도 보정은 하지 않는다.
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
실기 템플릿의 출발 방향·출발 동작·골은 후보 값이며 `field_verified=false`이므로 현장 확인 없이 실행되지 않는다.

## 3. 연결할 때 Pinky IP가 필요한가

**MQTT 명령 전송에는 Pinky IP가 필요하지 않다. 관제와 Pinky 모두 같은 브로커 주소로 접속한다.**
대상 선택은 site_id + robot_id + binding_id로 한다. client_id는 접속 세션 식별자이며 robot_id와 역할이 다르다.

| 정보 | 넣는 곳 | 의미 |
| --- | --- | --- |
| 브로커 IP/DNS | 양쪽 ROPORT_MQTT_HOST | Mosquitto가 실행 중인 PC 또는 EMQX 접속 주소 |
| 포트 | ROPORT_MQTT_PORT | 환경변수 미지정 시 모듈 기본 1883. 생성 키트의 broker.env.example은 18883을 명시하므로 양쪽에 같은 값을 적용. TLS 예시 8883; 실제 리스너와 일치 |
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
속도는 m/s이며 MQTT schema의 형식상 범위는 0.01~0.2다. 현재 Pinky 시험 계획은 별도로 0.015~0.08만 허용하고 기본값은 0.06이다. schema 예시의 0.1을 현재 실행 설정에 그대로 넣지 않는다. 이 범위가 현장 안전 속도 검증 완료를 뜻하지는 않는다.
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

1. `single/hardware` 또는 `two/hardware`를 선택한다. 모든 route의 `start_heading_rad`와 `start_maneuver`를 현장 값으로 확인한다. 1대 to_delivery는 핑키 담당 확인 자세 -1.5690 / none과 관제 속도 제한 0.06 m/s가 들어간다. N11 출발 후보와 목표점은 현장 검증이 필요하다. N11 후진/회전, 골·지도 정합성, 구동권 이전과 독립 watchdog을 확인한다.
2. 검증 완료 후에만 `field_verified=true`로 설정한다. 관제와 모든 Pinky에 같은 JSON을 배포하고 공통 LAN 브로커 주소·포트·TLS/계정을 설정한다.
3. PR #35의 ROS 패키지·trial_guard·trial launch와 PR #38의 지도·그래프를 Pinky README 절차로 준비한 후 아래 실행기로 연결한다. `--check`는 식별·모드·브로커 설정 형태만 검사하고 드라이버 import, 네트워크, 실제 준비 여부는 검사하지 않는다.
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

### 7.4 원본 웹 UI에서 1대/2대 주행 시험

관제 코드에서 연결하며 통합관제·Pinky 탭은 같은 수신 상태와 조작 패널을 사용한다. Pinky PR #35 드라이버에 PR #38의 지도·그래프를 반영하고 새 MQTT 키트·trial-v1 메시지를 쓴다.
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
export ROPORT_TRIAL_LAYOUT="$(realpath ../roport-pinky/src/roport_gz_world/layout/roport_airport.yaml)"
export ROPORT_TRIAL_SOCKET="$HOME/roport-trial/scenarios/single/hardware/trial.sock"
read -rsp '웹 시험 조작 키 (24자 이상): ' ROPORT_TRIAL_TOKEN
export ROPORT_TRIAL_TOKEN
uv run uvicorn roport_control_server.adapters.api.app:app \
  --host 127.0.0.1 --port 8000 --ws-max-size 4096 --ws-max-queue 4
~~~

`http://127.0.0.1:8000/ui/`를 연다. 시험 소켓이 설정되면 같은 원본 UI에서 자동으로 TRIAL 입력을 선택한다(`?mode=trial`도 호환). 조작 키를 입력한 다음 버튼을 클릭한다. **Enter만 눌러서는 출발하지 않는다.** **출발 → 입고 더미 승인 → 카트 적재 더미 승인**은 각각 현재 단계에서만 가능하다. `--scope line-only`는 N12 도착·정지 응답을 받으면 완료하며 mode_switch/navigate를 보내지 않는다. Nav2까지 검증할 때는 새 이력 파일로 `--scope nav2`를 명시한다(생략 시 기존 nav2 동작). scope는 관제 이력에 저장하므로 같은 파일의 시험 범위를 변경할 수 없다. 양쪽 trial.json과 MQTT 스키마는 바꾸지 않는다. 토큰을 사용자가 복사할 필요 없이 현재 `stop_session_id`를 보내며, 오래된 탭/중복 클릭은 실행부가 거부한다. 조작 키는 로그인 계정 시스템을 대신하지 않는다. 원격 접속은 인증된 사설망·HTTPS 리버스 프록시 등 배포 보호가 있는 환경에서 구성하며 무인증 공개 인터넷에 노출하지 않는다.

- 지도·OMX 설치 배치·탭은 기존 UI를 유지한다. Pinky는 고정 3자리 중 trial.json에 배정된 기체만 제어하고, 미배정 자리와 OMX 상태는 미수신으로 표시한다. 화면의 등록 편집기로 실기 시험 binding을 변경하지 않는다.
- 화면을 닫거나 API만 재시작해도 실행부는 계속된다. 새로 열면 같은 작업 상태를 받는다. 실행부 재시작은 기존 HOLD 규칙을 유지한다. 기존 journal에 HOLD가 있으면 웹에서도 자동 출발/초기화하지 않는다.
- `시험 중단 (보류)`는 후속 명령·lease 갱신을 막으며 조작 키가 필요하다. 무인증 중단은 팀 권한 정책 확정 전까지 적용하지 않는다. 즉시 물리 정지를 보증하는 비상정지 버튼이 아니다. 실제 정지는 Pinky watchdog·현장 정지 장치로 확인한다.
- 지도는 마지막 확인 노드 또는 Nav2 완료 응답의 위치를 표시한다. 연속 위치 송수신은 후속이며 Nav2 주행 중 위치를 추측하지 않는다. 화면 지도 해시/노드 그래프가 시험과 다르면 표시·출발·더미 승인을 막는다.
- `mode=demo`는 기존 화면별 모의 재생이고 `mode=trial`은 실제 MQTT 시험이다. 가상 시험도 동일 TRIAL UI를 사용하되 `single/simulation`·loopback 브로커·`mock_pinky_trial.py`를 사용한다. 실물용 실행부와 연결하지 않는다.
- 소켓은 사용자 전용 권한으로 만들고 Ctrl+C/SIGTERM 종료 시 제거한다. SIGKILL·전원 차단으로 파일이 남으면 소유 프로세스 종료를 확인한 뒤 소켓 파일만 정리한다. 실행 중 소켓이나 journal을 삭제해 잠금/복구를 우회하지 않는다. Unix 소켓 경로는 107바이트 이하로 사용한다.

실물(`mode=hardware`) 명령과 계획 설정은 **접속자와 URL 모두 loopback이거나 서버가 HTTPS로 인식한 요청**만 허용한다. 원격 HTTP에서는 조회만 가능하며 키 입력과 조작을 차단한다. HTTPS 프록시는 외부 Host/Origin을 유지하고 Uvicorn `--forwarded-allow-ips`에 신뢰할 프록시 주소만 지정한다. 클라이언트의 `X-Forwarded-Proto`만으로 우회할 수 없다.

레이아웃은 `ROPORT_MAP_LAYOUT`이 우선이고 미설정 시 `ROPORT_TRIAL_LAYOUT`을 사용한다. HTTP/WS는 worker당 200ms snapshot 캐시를 공유하며 명령 판단은 새로 조회한다. 실행부가 늦게 연결되거나 계획이 바뀌면 수신 경로선을 갱신한다. 기동 중 잠금·소켓·브로커 준비 실패 시 미출발 READY 이력은 유지하며, 진행 중 이력과 기존 HOLD는 자동 초기화하지 않는다. 웹 명령의 MQTT 연결 오류·전송 시간 초과는 결과 불명 HOLD로 남기고 자동 재전송하지 않는다.

HOLD의 최초 오류는 이후 운영자 종료·브로커 종료로 덮어쓰지 않는다. 오류가 있는 status 응답도 관측 정보로 저장하지만 완료·진행 성공 응답의 오류는 계속 거부한다. HOLD에서는 lease 갱신을 재개하지 않는다.

### 7.5 2대 원본 UI 시험 · 실물 직전 준비

7.4와 같은 실행 명령을 사용하고 `TRIAL_DIR`과 `ROPORT_TRIAL_SOCKET`을 **같은 `two/hardware` 폴더**로 맞춘다. 시험 중 1대/2대 설정을 바꾸지 않는다. 기존 1대 설정·이력은 보존하고, 시나리오별 site/control/binding과 각 프로세스의 새 journal을 분리한다. 한 시험의 JSON은 관제·두 Pinky에 동일본을 복사하며 `sha256sum trial.json`으로 대조한다.

| 순서 | 담당·경로 | 웹에서 다음 단계 승인 |
| --- | --- | --- |
| 1 | 물류 pinky1: N16→N14→N02→N10→N18→N01 | 시나리오 출발 → OMX 창고 상차 완료 더미 |
| 2 | 물류 pinky1: N01→N18→N07→N23→N19→N11 | OMX 입고 완료 더미 |
| 3 | 물류 pinky1: N11→N19→N23→N07→N05→N15→N14→N16 | 창고 복귀·정지 확인 후 물품 요청 |
| 4 | 스마트카트 pinky3: H1→N09→N24→N19→N11 | OMX 카트 적재 완료 더미 |
| 5 | 스마트카트 pinky3: N11→N19→N24→N09→N08→N12 | line-only 완료 / nav2는 전환·목표 결과 대기 |

물품 요청은 설정의 `package_id` 1개만 허용한다. 잘못된 수량·기체·binding·이전 단계 요청은 거부한다. 물류 핑키가 돌아오기 전에 카트를 진입시키지 않는다. 두 기체의 상태가 신선하고 정지·line 제어 해제가 확인돼야 수동 승인이 가능하다. 출발 시 두 초기 위치(N16/H1), 요청 시 창고 복귀(N16)와 카트 위치(H1)를 실행부가 다시 검사한다. 한쪽 상태 유실·재시작은 전체 시험을 HOLD로 만들며 자동 복구하지 않는다.

**설정 후보와 검증 근거** — 전 구간 `start_maneuver=none`, `speed_limit_mps=0.06`, 단계 제한 300초다. Pinky 드라이버는 현재 line_nav 기본 속도와 이 제한 중 작은 값을 사용하므로, 실제 증속은 Pinky 기본 속도도 함께 맞춰야 한다. Nav2 속도는 별도 launch 설정이다. `none`은 로봇의 기존 상차 완료 회전을 생략한다는 뜻이 아니다. N01/N11에서 다음 route를 받은 드라이버가 `load_done`을 보내 회전 후 출발한다.

**0.08 m/s 물류 시험 선택** — PR #43의 노드 접근 감속·주행 보정이 포함된 Pinky 설치본에서 검증할 경로의 UI 속도 상한(또는 `trial.json` 해당 route)을 `0.08`로 지정하고, Pinky `trial.launch.py` 실행 인자에 `line_speed:=0.08`을 명시한다. launch 허용 범위는 0.02~0.08이고 미지정 기본값은 기존 0.03이다. 관제·로봇 양쪽 값을 확인해야 하며 낮은 속도로 실행한 뒤 로봇의 현재 base 속도가 낮아졌다면 상한만 올려도 자동 증속되지 않는다. 정지 후 로봇 실행 설정을 다시 맞춘다. 공통 생성값·자동 출고 계획은 0.06을 유지하며 스마트카트·Nav2를 함께 증속하지 않는다. PR #43의 특정 기체 시험 결과를 다른 기체·관제 trial 경로 검증으로 간주하지 않는다.

| route | start_heading_rad | 근거·현장 확인 |
| --- | --- | --- |
| to_loading | -1.5690 | 1대 시험의 동일 N16 출발 값 재사용; N01 진입은 새 시험 |
| to_delivery | 0.0017453293 | N01 도착 후 반시계 90° 상차 대기 자세; 드라이버/2D 시뮬레이터 대조 |
| return_warehouse | 0.0017453293 | 검증된 N11 상차 대기 자세 재사용; 창고 복귀 경로는 새 시험 |
| cart_to_delivery | 1.5725416560 | H1→N09 북향 + 맵 회전 0.1°; 두 번째 기체 배치 확인 필요 |
| cart_to_switch | 0.0017453293 | 1대 실물 시험의 출발 자세 재사용; v14의 N24 경유와 두 번째 기체에서 확인 필요 |

생성한 hardware는 값이 채워져도 `field_verified=false`다. 네트워크나 journal을 만들지 않는 관제 정적 검사는 아래와 같다. 통과는 **물리 준비·브로커 연결·Nav2 검증 완료를 뜻하지 않는다**.

~~~bash
uv run python -m roport_control_server.executor.pinky_trial --config "$TRIAL_DIR/trial.json" --scope line-only --check
~~~

실물 전 확인:

1. 두 Pinky의 같은 맵·그래프·driver/guard 버전과 배치(N16 남향, H1 북향), N01/N11 상차 후 자세, 변경한 속도에서 센서·후진·회전·독립 정지를 확인한다. 이전 0.03 m/s 시험을 0.06 m/s 검증으로 간주하지 않는다. 검증한 범위는 우선 `line-only`이며 Nav2는 별도 시험이다.
2. 두 PC의 **모든 ROS 노드와 MQTT runner에 각각 동일한 로컬 ROS_DOMAIN_ID를 적용하고, 두 기체 사이에는 서로 다른 값을 사용**한다(예: 물류 31 / 카트 32). 현재 driver의 `line_nav/*`·구동 토픽을 동일 LAN에서 교차 수신하지 않도록 한다. 기존 팀 배정값이 있으면 그 값을 쓴다.
3. LAN 브로커 host/port/TLS를 세 PC에서 일치시킨다. Pinky IP를 관제 MQTT 명령에 넣지 않는다. 역할은 `--role logistics` / `--role smartcart`, 슬롯·binding은 동일 JSON으로 구분한다. 두 runner의 시작 노드는 각각 `ROPORT_PINKY_START_NODE=N16` / `H1`이다.
4. 현장 확인 후 hardware JSON을 최종 확정·재배포하고 역할별 새 journal로 실행한다. PR #35 driver·guard와 공통 runner에 **PR #38의 지도·그래프 코드를 함께 반영하고 로봇 workspace를 다시 빌드/source**한다. MQTT JSON Schema는 그대로이며 `field_verified=true`와 `--hardware-authorized`는 현장 확인 후 적용한다.

지도 v14에서는 `graph_version=line-v14-20261106`, `map_version=4656a67672d005a5`다. v13 설정·성공 기록은 보존하고 새 폴더에 설정을 생성한다. 관제와 로봇 모두 같은 새 JSON을 써야 하며 기존 journal을 재사용하지 않는다. N23·N24는 경유 노드이고 이번 시험의 인도장 정차점은 계속 N11이다. N21·N22 정차 시나리오는 포함하지 않는다. PR #39의 선반 마커(28~33)는 이번 더미 OMX 시험의 완료 조건에 사용하지 않는다.

배포 때는 현재 소스에서 생성한 키트를 사용한다. 이전 키트에는 관제 통합 과정에서 반영한 MQTT 수신 시각/상태 전송 확장과 오류가 담긴 status 응답 검증 수정이 빠져 있을 수 있다. 메시지 스키마가 같아도 파일 버전까지 같다는 뜻은 아니므로 `manifest.json`의 파일 해시를 함께 확인한다. 시험 브로커 포트 기본값은 1대 실물 시험과 동일한 **18883**이며 실제 listener 설정과 대조한다.

브라우저의 물품 요청은 `POST /api/trial/commands`의 `action=request`, `items=[{package_id, quantity:1}]`로 실행부에 전달된다. MQTT에는 기존 `route.requested`가 나가므로 새로운 로봇 메시지가 생기지 않는다. 새 UI에는 배정 역할·기체별 노드·진행률을 표시하며 스마트카트 Nav2 완료 좌표를 물류 핑키에 적용하지 않는다.

### 7.6 스마트카트 역할만 시험

관제 실행부에 `--smartcart-only`를 추가한다. 기존 `two/hardware/trial.json`을 그대로 사용하며 기본 배정은 smartcart=pinky3다. 물류 기체 상태·주행·창고 복귀·물품 요청은 실행하지 않는다. 창고 작업·인도장 입고는 이번 시험에서 완료됐다고 가정하며, 실제 물류 완료 이벤트나 이력을 만들지 않는다.

진행은 **H1 → N09 → N24 → N19 → N11 → 카트 적재 완료 더미 → N19 → N24 → N09 → N08 → N12 → Nav2 전환 완료 → 공용공간 목표 도착**이다. `--scope line-only`를 선택하면 N12 정차에서 종료한다. 출발·적재 더미는 웹 버튼 또는 기존 CLI 입력으로 수동 승인한다. 시작 시 H1 위치·정지·라인 모드와 해당 기체의 최신 상태를 확인한다.

~~~bash
uv run python -m roport_control_server.executor.pinky_trial --config "$TRIAL_DIR/trial.json" --journal "$TRIAL_DIR/control_cart_$(date +%Y%m%d_%H%M%S).json" --socket "$TRIAL_DIR/trial.sock" --smartcart-only --scope nav2 --hardware-authorized
~~~

Pinky는 같은 JSON의 `--role smartcart`, `ROPORT_PINKY_START_NODE=H1`로 기존 runner·driver를 사용한다. 로봇 MQTT 모듈·JSON Schema 변경은 없다. 단독 시험 여부는 관제 이력에 고정되므로 전체 2대 시험과 같은 journal을 재사용하지 않는다. 전체 시험과 control_id·브로커가 같으므로 기존 실행부를 종료한 후 시작한다. API 코드도 업데이트했다면 웹 서버를 재시작하고 화면을 새로고침한다. 화면은 배정된 스마트카트 한 대만 감시하며 물류 슬롯은 시험 미배정으로 표시한다.

### 7.7 공통 실행부와 구간/작업 설정

**공통 two_pinky 설정으로 한 번 실행하고 UI에서 1대·2대·스마트카트를 선택한다.** 관제 실행 명령에 `--smartcart-only`를 넣지 않고 기본 scope(nav2)를 사용한다. `--scope line-only`와 `--smartcart-only`는 기존 제한 실행용으로 보존하며, 해당 제한은 UI에서 해제할 수 없다. single_pinky 설정에는 1대 기본안만 표시한다.

```bash
# 현장 확인을 마친 공통 two/hardware 폴더를 한 번 지정한다.
export TRIAL_DIR="/absolute/path/to/scenarios/two/hardware"
source "$TRIAL_DIR/broker.env"
env -u PYTHONPATH uv run python -m roport_control_server.executor.pinky_trial \
  --config "$TRIAL_DIR/trial.json" \
  --journal "$TRIAL_DIR/control_ui_$(date +%Y%m%d_%H%M%S).json" \
  --socket "$TRIAL_DIR/trial.sock" --hardware-authorized
```

FastAPI는 기존 명령에서 `ROPORT_TRIAL_SOCKET`을 위 소켓으로 지정한다. 관제 실행부·API·브로커를 켠 채 기본안을 바꿀 수 있다. READY 또는 COMPLETE에서만 적용할 수 있으며 HOLD는 원인 확인 후 새 journal로 시작한다.

| UI 기본안 | 참여 기체 | 기본 동작 |
| --- | --- | --- |
| 1대 시험 | logistics=pinky1 | N16에서 N01 상차장을 우회해 N11 → 입고·적재 더미 → N12 |
| 2대 시험 | logistics=pinky1 + smartcart=pinky3 | 물류 상차·입고·창고 복귀 → 물품 요청 → 카트 인도장 진입·적재 → N12 |
| 스마트카트 단독 시험 | smartcart=pinky3 | H1 → N11 → 적재 더미 → N12 |

**N12 이후 Nav2 목표까지**를 선택하고 기본안을 불러오면 모드 전환·목표 이동 단계도 포함한다. 목표는 공통 trial.json의 nav_goal을 읽고 UI에서 편집할 수 있다. 기본안을 불러오는 동작은 편집창만 바꾸며 적용/출발을 하지 않는다.

모든 기체는 같은 two/hardware 설정의 site/control/binding을 사용한다. 예전 single/hardware 설정으로 핑키를 실행했다면 공통 JSON을 한 번 복사해 새 robot journal로 실행해야 한다. 이미 two/hardware로 연결한 핑키는 코드·메시지 스키마 변경이 없다. 핑키1은 logistics, 핑키3은 smartcart 역할로 실행한다. 실제 출발 노드와 방향은 현장에서 맞춰야 하며 UI 선택이 로봇 위치를 바꾸지는 않는다.

적용한 계획에 등장하는 기체만 폴링·준비 상태·timeout·점유 검사 대상으로 삼는다. 스마트카트만 선택하면 물류 핑키 연결 없이 시작할 수 있다. **비참여 기체는 시험 통로 밖에 정지시킨다.** 미참여 기체를 자동 감지/회피하는 기능은 없다.

1. 기존 실행부와 API를 새 코드로 재시작한다. 최초에는 새 관제 journal을 사용한다. 공통 two/hardware의 연결 식별자를 유지한다. 예전 single 설정을 사용하는 로봇은 위 전환 절차를 따른다.
2. 웹의 조작 키를 입력하고 **시나리오 선택 · 노드 편집**을 펼친다. 기본안 불러오기 또는 현재 계획 복제 후 작업을 추가/삭제/위로 이동한다. 단계별 기체·노드 목록·출발 방향(rad)·속도 상한(m/s)·Nav2 목표(map m/rad)를 지정한다. 노드는 공백·쉼표·화살표로 구분해 직접 입력한다. 첫·끝 노드로 최소 간선 경로를 계산할 수도 있다. 존재하지 않는 노드, 끊긴 연결, 단계 사이 위치 불일치는 적용 전에 거부한다.
3. 설정 검증 후 이름으로 서버에 저장한다(선택). 실물에서는 해당 계획의 시작 위치/방향, 비참여 기체·통로, watchdog을 확인하고 체크한다. **실행 계획 적용**은 주행하지 않는다. 적용 후 최신 상태의 출발 가능 여부를 확인하고 **시나리오 출발**을 따로 누른다.
4. 더미 승인은 현재 정차 세션에만 유효하다. 시작 후 계획 변경은 거부한다. COMPLETE 뒤 다른 계획을 적용하면 이전 전체 기록을 `run-<task_id>.json`으로 보관하고 새 task_id로 READY가 된다. HOLD·재시작 상태는 현장 대조 후 새 journal로 준비한다.

저장된 정의는 관제 journal과 같은 폴더의 `scenario-plans.json`(이름 기준 최대 16개)에 보관한다. 실행본·계획 revision·현장 확인·시험 가정은 journal에 고정하며 브라우저 새로고침으로 사라지지 않는다. 저장만 한 편집본은 실행에 반영되지 않는다. 미적용 편집이 있으면 출발 버튼은 비활성화해 이전 계획의 오출발을 막는다. 다른 화면이 계획을 바꾸면 기존 편집창의 적용은 거부하므로 현재 계획을 다시 복제한다.

| 항목 | 현재 지원 / 제한 |
| --- | --- |
| 실행 단위 | route, gate(더미), request(운영자 물품 요청), mode_switch, navigate; 최대 24단계, 순차 실행 |
| 노드 경로 | 실제 그래프 연결·연속성·기체별 모드 검사; 경로 중간 U턴과 정차선 노드 통과 금지 |
| 정차 목적지 | 현재 v14: N01, N11, N12, N16, N17, N21, N22, H1, H2. N19 등의 교차로는 경유만 가능 |
| 시작 위치·방향 | UI는 물리 위치/heading을 변경하지 않는다. 기존 runner의 START_NODE와 실제 자세가 맞아야 한다. loading 이후 방향과 일반 구간 첫 방향은 Pinky 드라이버가 최종 검증 |
| 속도·모드 | 계획 허용 범위 0.015~0.08 m/s, 기본 0.06; 실제 속도는 로봇 로컬 설정과 함께 결정. start_maneuver=none, N12 line→Nav2만 지원 |
| 부분 작업 | 선행 물품 상태는 시험 가정으로 기록. 실제 OMX 완료로 위조하지 않음. 물품은 기존 설정 1개, 수량 1 |
| 안전 범위 | 설정된 시험 기체의 노드 점유와 단일 명령 실행. 비참여 로봇·사람·장애물·구간 전체 예약은 자동 감지하지 않으며 현장 확인 필요 |

**고객 요청 시험:** 같은 패널에서 요청 ID, 자동/지정 역할, 물품 위치, 목적 정차 노드, 적재 후 출발 방향, Nav2 포함 여부를 지정한다. `delivery_plan`이 현재 계획에 참여한 기체의 준비된 시험 상태로 기체와 최소 간선 경로를 선택하고 적재 더미 대기를 삽입한다. 계획을 검토해 적용할 때 상태가 바뀌었으면 재생성해야 한다. 같은 요청 ID/내용은 중복 실행하지 않으며 다른 내용의 같은 ID는 거부한다. 현재 위치와 목적지를 바꾸면 경로가 달라지지만, 실행 중 자동 재계획·WSS 수신·재고 조회·물품 준비 시각 조정·다중 주문 대기열·동시 로봇 이동은 구현 범위 밖이다.

코드 경계: `executor/plans.py`는 입력→검증된 순차 계획, `pinky_trial.py`는 공통 명령/완료/복구 실행, `trial_socket.py`는 단일 owner RPC, `adapters/api/trial.py`는 상태 투영/지도 검사, `spatial_assets/scenario.js`는 편집/미리보기다. 향후 서비스 요청 어댑터는 계획 생성 입구에 연결하며 MQTT 수행 코드를 복제하지 않는다. 실제 OMX 공동 작업은 두 팔·구역 예약 및 제어 PC 완료 계약이 추가되기 전까지 더미 승인이다.

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
| src/roport_control_server/executor/pinky_trial.py | 관제 CLI/상태 전이 |
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

## 10. 실물 드라이버를 사용하는 가상 시험 환경

`mock_pinky_trial.py`는 빠른 메시지 검사용이다. 실제 주행과 연결된 시험에는 아래 환경을 사용한다.
**원본 웹 UI → FastAPI → 동일 관제 실행부 → 실제 loopback Mosquitto → 동일 MQTT runner →
Pinky의 `TrialDriver`·`TrialLogic`·`RouteExecutor` → 가상 IR/odom/모터**로 연결한다.
팀원 소스를 수정하거나 복사하지 않고 현재 checkout에서 불러온다. ROS 설치 없이 실행하며 wheel에는 포함하지 않는다.

```bash
# services/roport-control-server에서. Mosquitto가 PATH에 없으면 MOSQUITTO_BIN 지정.
env -u PYTHONPATH uv run python tools/run_virtual_trial.py
# 기본: UI에서 1대/2대/스마트카트 선택. 아래는 기존 제한 실행 옵션.
# 2대: logistics=pinky1 / smartcart=pinky3
env -u PYTHONPATH uv run python tools/run_virtual_trial.py --scenario two
# 스마트카트만 H1 → N11 → N12 → 모의 Nav2 목표
env -u PYTHONPATH uv run python tools/run_virtual_trial.py --scenario smartcart
```

1. 터미널에 출력된 `url`을 브라우저에서 연다. 임의의 빈 포트를 써 기존 8000·현장 브로커와 충돌하지 않는다.
2. 출력된 `key_file`을 `cat`으로 확인해 웹 화면의 **시험 조작 키**에 입력한다.
3. 기존 출발·더미 완료·물품 요청 버튼 또는 시나리오 설정을 사용한다. 기본 실행은 1배속이며 자동 승인은 없다.
4. 종료는 실행 터미널에서 **Ctrl+C**. 자식 프로세스도 종료하고 모든 기록은 `~/roport-trial/virtual/<실행별 폴더>/`에 남긴다.

**가상 시험 초기화** 버튼은 조작 키 입력 후 사용할 수 있다. 가상 주행 중·완료·보류 상태에서
실행기의 가상 프로세스들을 종료하고, 로봇 위치·계획·binding을 실행 당시 초기값으로 새로 만든다.
편집한 계획도 초기화되며 자동 출발하지 않는다. 같은 URL·조작 키로 재연결되고, 이전 기록 전체는
기존 실행 폴더 이름 뒤에 `-before-reset-…`가 붙은 형제 폴더에 보존한다. 현재 `run.json`의
`previous_run_dir`로 직전 기록을 찾을 수 있다. 실물 시험과 개별 수동 실행에는 이 버튼을 제공하지 않는다.

`--verify`를 추가하면 가상 상하차·물품 요청을 HTTP API로 자동 승인해 끝까지 검증한 뒤 종료한다.
실물 명령은 발행하지 않는다. 실행마다 site/control/binding·journal을 새로 만들며, 기존 `ROPORT_*` 환경값을
자식에게 넘기지 않는다. MQTT·웹 서버 모두 127.0.0.1에만 바인딩하고 DB는 disabled다.
실패한 실행을 이력 수정으로 재개하지 말고 원인을 확인한 뒤 새 실행을 만든다.

### 실물 완료 이력의 설정 가져오기

```bash
env -u PYTHONPATH uv run python tools/run_virtual_trial.py \
  --scenario two --record /absolute/path/to/hardware/control_completed.json
```

`--record`는 hardware 관제 journal의 설정을 읽어 경로·출발 방향·제한 속도·deadline·Nav2 목표를 유지한다.
연결 ID와 hardware 승인은 가상 시험용으로 바꾸고 원본 파일은 수정하지 않는다. 현재 지도/graph/node/scenario와
다른 기록은 거부한다. v13 실물 기록을 v14에 그대로 재생하지 않는다. `smartcart`는 two 기록을 사용할 수 있다.
기록 미지정 시 현재 v14 경로와 가상 시험 기본 속도 0.08m/s를 사용한다. 이 기본값은 실물 설정을 변경하지 않는다.

- `run.json`: 시뮬레이션 조건, 원본 기록 SHA256, 실제 accepted→completed 구간 시간, 실행한 코드·지도 해시, PID.
- `control.json`, `logistics.json`, `smartcart.json`: 실제 MQTT 명령·응답 및 상태 전이 이력.
- `*.log`, `outcome.json`: 프로세스 진단과 자동 검증 결과. `ui-key.txt`는 로컬 비밀 파일이며 공유하지 않는다.

실물과 가상의 accepted→completed 시간을 비교할 수 있지만, 실물의 도착 시간을 강제로 재생하지는 않는다.
실제 명령에 따라 현재 드라이버가 IR를 판독하고 경유·회전·정차를 계산해야 구현 오류가 드러나기 때문이다.

### 재현 범위와 실패 시험

| 항목 | 적용 범위 |
| --- | --- |
| 명령/응답·중복·ACK·재시도·boot/binding·정차 판정 | 실제 MQTT 모듈과 Pinky 드라이버 공통 코드 |
| 경로·상하차 90° 회전·load_done 후 복귀·창고 후진 정차·N12 handoff | 현재 실측 맵 + 실제 라인 실행 코드 |
| 센서·바퀴 | 팀의 2D `Sim`: 20Hz, IR 3채널, 선폭·센서 위치, 이상적인 odom. 기체별 재측정값은 아님 |
| 속도 | 실제와 같은 `min(로컬 base_speed, 명령 상한)` 계산. 가상 기본 프로필은 두 값을 모두 0.08m/s로 맞추며, 가상 실행기의 최대 속도도 이 프로필을 허용하도록 설정. `--record`는 기록의 제한 속도를 사용 |
| Nav2 | `--nav2 simulated` 기본: 프로필 속도(기본 0.08m/s)로 장애물 없는 가상 목표 이동. 실제 드라이버의 전환/완료 검사를 사용하나 Nav2·AMCL·경로 계획은 실행하지 않음 |
| 구동권 watchdog | 가상 프로세스 내 1초 lease 모델. 실제 ROS 독립 guard 및 프로세스 강제 종료 시 물리 정지를 검증한 것은 아님 |

- `--scope line-only`: N12 정차까지만 검증.
- `--nav2 off`: 준비되지 않은 Nav2로 전환/출발이 진행되지 않는지 확인.
- `--fault line-loss`: 해당 로봇의 누적 주행 5초 후 가상 테이프를 제거, 실제 라인 유실 오류와 HOLD 확인.
- `--fault odom-stale`: 5초 후 odom 갱신 중단, 정차 완료를 인정하지 않고 최종 timeout/HOLD 확인.

Nav2 장애물 회피·실제 Wi-Fi 손실/지연·기체별 센서 편차·마찰·배터리·하중은 재현하지 않는다.
모의 Nav2 성공이나 실제와 비슷한 주행 시간은 실물 Nav2 성공 증거가 아니다.


## 11. 실시간 위치 표시 · DB 없이 연결

`TrialDriver.observe_motion()` → 이식 MQTT runner → `roport/v1/{site}/robot/{pinky1|pinky2|pinky3}/motion` → 관제의 별도 수신 연결/메모리 최신값 → Unix socket → FastAPI → 원본 웹 UI. 기존 주행 명령·완료 응답 `trial-v1`과 `trial.json`은 변경하지 않는다.

| 항목 | 규격/처리 |
| --- | --- |
| Envelope | `schema_version=1.1`, `profile=roport-state-dev-v1`, `type=motion`, `message_id`, `site_id`, `source=기체 ID`, `target=control`, `sent_at` |
| Payload 식별 | 같은 시험의 `binding_id`, runner와 같은 `boot_id`, 증가하는 `sequence`, 실제 관측의 `observed_at` |
| Payload 위치 | `map_id`, `map_version`, `frame_id=map`, `pose_source`, `pose={x,y,z,yaw}`(m/rad) 또는 null, `velocity=null`, `joints={}` |
| 출처 | `tf`: 로봇의 유효한 지도 TF(계산 시각이 아닌 TF 근거의 경과 시간 반영); `line_estimate`: 실측 원점 + 상태 heading과 같은 frame_yaw의 표시용 pose_est; `simulation`: 가상 센서 위치 |
| 주기/전달 | 5 Hz, 정지 중에도 유효 관측 발행. QoS0, retain=false, MQTT expiry 1초. 작업 ACK/재전송/고빈도 journal 저장 없음 |
| 수신 검증 | 현재 시험 site/binding, 상태 응답으로 확인된 boot, 순번/측정 시각, map/frame 검증. 1초 초과 표본·역순·중복·retained는 폐기 |
| 화면 | 상태 스트림 약 10 Hz. 수신 좌표 사이만 최대 0.2초 보간하며 외삽하지 않음. 모드 전환·무효 위치·수신 단절 시 마지막 위치/경과 시간 표시. 다른 boot/지도 위치 재사용 금지 |

이 motion 표본은 준비·도착·정지·자원 점유·다음 단계 승인에 사용하지 않는다. 위치 수신 연결과 작업 명령 연결의 큐를 분리해 표시 입력의 오류·적체가 작업 이벤트를 소비하지 않게 한다. 관제 프로세스가 재시작되면 메모리 위치는 새 표본을 기다리며, 운영 DB/보류된 registry 기능은 필요하지 않다.

**실물 배포:** 관제 코드와 Pinky의 `roport_pinky/trial_driver.py` 및 MQTT 모듈(`motion.py`, `mqtt.py`, `pinky_trial.py`, `trial_runner.py`)을 함께 갱신한다. 재생성한 이식 키트에도 `motion.py`를 포함한다. 기존 실행 명령/브로커/기체 ID/binding/trial.json은 유지한다. ACL을 사용하는 현장은 기체의 자기 motion 토픽 발행과 관제의 해당 토픽 구독을 허용한다. 기존 드라이버에는 선택 hook이 없으므로 작업 시험은 계속 가능하지만 연속 위치는 표시되지 않는다. 실물 주행은 업데이트 후 별도 재검증해야 한다.

위치 시각 보정에는 Pinky의 `trial_nav2.py`도 함께 갱신한다. 라인 추정은 현재 노드를 알며 STOPPED·ERROR가 아닐 때만 사용하고, HOLD·safe_stop 뒤에는 null을 발행해 이전 좌표를 마지막 확인 위치로 표시한다. 유효한 지도 TF는 정지 중에도 사용할 수 있다. 표시 hook의 예외는 작업 루프로 전파하지 않고 집계하며 첫 오류와 50건마다 예외 종류를 로그에 남긴다. 웹 시험 패널의 `위치 수신 거부 누적`에는 시각 지연/시계 차이·중복/역순·기체 식별 불일치·잘못된 표본의 건수를 표시한다. 시각 사유가 있으면 PC와 로봇 시계를 동기화하며 허용 시간 기준을 완화하지 않는다. 이 진단은 관제 실행부 재시작 시 초기화되고 주행 승인에는 사용하지 않는다.

TF는 기존 `trial/localization_state`의 유효성 검사를 그대로 사용한다. Nav2/AMCL/TF가 준비되지 않은 라인 모드는 `pose_est`를 실측 frame의 cm→m, 회전/이동 변환으로 표시한다. 이 추정은 바퀴 축 위치의 근사이며 정밀 localization을 대체하지 않는다. 가상 Nav2도 실제 ROS Nav2의 장애물 회피 검증을 대체하지 않는다.

## 12. 기존 Pinky 브로커에 OMX1·2 연결

관제 owner 한 개가 같은 `ROPORT_MQTT_*` 연결로 Pinky 주행과 `shelf-01` 작업을 보낸다.
OMX PC에서 별도 브로커를 열거나 Pinky runner를 OMX용으로 복제하지 않는다.
서비스 입고의 `WAIT_INBOUND` → `shelf_inbound.requested`, 출고의 `WAIT_CART_LOAD` →
`shelf_outbound.requested`다. OMX3 창고 상차는 아직 수동 완료다.

### 1. 사용 중인 설정에 OMX만 추가

작업을 종료하고 관제 owner를 내린 뒤, 관제 서비스 디렉터리에서 실행한다.
`TRIAL_DIR`은 현재 사용하는 trial.json 디렉터리다. 기존 파일을 직접 덮어쓰지 않는다.

```bash
uv run python tools/configure_omx_trial.py \
  --config "$TRIAL_DIR/trial.json" --output "$TRIAL_DIR/trial-omx.json" \
  --execution-mode mock --slot 2b \
  --model-version test-model --calibration-version test-cal
```

이 명령은 연결하거나 구동하지 않는다. 기존 site/control/Pinky binding·경로·속도·현장 확인 값을
보존하고 `omx` 설정만 추가한다. 이미 OMX 설정이 있으면 binding도 보존한다. 출력 파일이 있으면
거부한다. 기존 관제 실행 명령의 `--config`만 `trial-omx.json`으로 바꾸고 **새 journal**을 사용한다.
Pinky에는 기존 trial.json을 유지할 수 있다(기체가 사용하는 값은 동일). 이전 owner와 함께 실행하지 않는다.

`mock`은 OMX 팔이 움직이지 않아도 시간 경과로 완료를 보내므로 **무구동 통신 시험용**이다.
실물 Pinky 작업에 이 완료를 물리 상하차 증거로 사용하지 않는다. 기존 `mode=simulation` 설정과
가상 Pinky로 먼저 검증하며, simulation owner는 기존 규칙대로 loopback broker만 허용한다.
서로 다른 PC의 수신 확인은 아래 status 조회를 사용하고 관제 출발을 누르지 않는다.

### 2. OMX PC에서 같은 브로커에 접속

관제·Pinky와 동일한 실제 `ROPORT_MQTT_HOST`, `ROPORT_MQTT_PORT`, TLS/인증 설정을 적용한다.
예제의 포트가 18883이어도 실제 브로커가 1883이면 모든 PC에서 1883을 사용한다.
`localhost`는 각 PC 자신을 가리킨다. 인증값은 JSON·명령 기록·저장소에 넣지 않는다.

```bash
cd /path/to/kernel-robot-repo-1/services/roport-omx
uv sync --locked
# 기존에 배포한 공통 broker.env를 사용한다. 새 브로커를 시작하지 않는다.
source /path/to/broker.env
```

설정 도구가 출력한 `uv run python scripts/cooperative_host.py ...` 명령을 OMX PC에서 실행한다.
`REPLACE_WITH_PERSISTENT_OMX_JOURNAL_PATH`는 사용자 홈 아래 새 영속 파일로 바꾼다.
출력 명령은 site/control/OMX binding을 trial-omx.json과 일치시킨다. Host client ID는 다른
MQTT client와 중복되면 안 된다. 모의 Host의 정상 로그는 `SIMULATED executor`와 `subscribed`다.

| 항목 | 관제와 OMX Host의 일치 조건 |
| --- | --- |
| site | `trial-omx.json.site_id` ↔ `--site` |
| 관제 ID | `control_id` ↔ `--control` |
| OMX 등록 | `omx.binding_id` ↔ `--binding`; Pinky binding과 별개 |
| 브로커 | host·port·TLS·CA·사용자 권한이 같은 접속 구성 |
| 작업 | slot·모델 버전·보정 버전이 현장 실행부의 실제 설정과 일치 |

ACL은 `roport/v1/{site}/workcell/shelf-01/{cmd,event,ack}`를 추가한다.
관제는 cmd/ack 발행·event 구독, OMX Host는 cmd/ack 구독·event 발행이다. 모두 QoS1,
retain=false. MQTT 연결 성공만으로 팔 준비가 확인되는 것은 아니다.

무구동 상태 확인은 도구가 함께 출력하는 `cooperative_control_sim.py status` 명령을 사용한다.
같은 broker.env를 읽고 `REPLACE_WITH_STATUS_LOG_PATH`를 로컬 기록 경로로 바꾼다.
이 기존 조회 스크립트는 TLS/인증 환경변수를 읽지 않으므로 무인증 시험 브로커에서만 사용한다.
실제 관제 owner를 함께 실행하지 않는다. 정상은 `status.completed` 왕복이며 작업은 시작하지 않는다.

### 3. 실물 실행부에 연결할 때 필요한 조건

main의 PR45 Host는 mock만 제공한다. 실물 제어판 어댑터는 FFT1-158의
`feature/taegyeong/omx-cooperative-panel`에 별도로 있으며 main 병합/현장 검증과 구분한다.
현재 확인본 `d398ba5`의 `PanelExecutor.readiness()`는 `arms={}`를 반환하므로 관제의 준비 검사에
통과하지 않는다. 이 상태에서 검사를 완화하거나 모의 완료로 우회하지 않는다.

실행부 담당자가 연결해야 할 경계는 기존 `roport_omx.cooperative.Executor`다.

- `readiness()`: 실제 armed·busy·error와 omx1/omx2 각각의 connected/returned/safe/grasping.
  작업 전송에는 최신 양팔 `(True, True, True, False)`가 필요하다. 과거 last_result를 현재 상태로 재사용하지 않는다.
- `start(task)`: operation·slot·package·model/calibration을 확인하고 한 작업을 시작한다.
  빠르게 반환해야 하며, 제어판이 모터와 카메라를 단독 소유한다.
- `observe()`: 같은 execution의 running/finished/failed와 실제 양팔 상태를 반환한다.
  정상 종료와 양팔 복귀·그리퍼 열림이 확인되어야 완료한다. 실패·결과 유실은 HOLD한다.

이 경계가 충족된 뒤 설정 도구에 `--execution-mode hardware`와 현장 모델/보정 버전을 지정한다.
도구가 출력하는 panel 명령에는 `--executor panel --routes ... --box-marker ...`가 추가된다.
`--routes`에는 `(INBOUND 또는 OUTBOUND, slot_id)`별 실제 모델 경로·작업 문장·episode_s·reset_s를
등록하고, 상자 마커는 실물 물품과 확인한다. package_id→마커 자동 매핑은 아직 없다.
제어판 v3.8+의 실제 실행 명령·시리얼/카메라·모델·보정 파일은 현장 담당자가 제공해야 한다.

UI 수동 완료와 MQTT 완료는 같은 정차 세션에서 한 번만 처리된다. 이미 작업을 보냈다면 수동 완료에도
접수 이후의 최신 Host 대기·양팔 복귀 정보가 필요하다. `arm_return_only`는 선반 안착·재고 확정이 아니다.

### Nav2 실패 보고와 같은 목적지 재시도

Pinky는 복구를 최대 2회 시도하고, 복구 중 실제 위치 진전이 없으면 30초 안에 실패한다.
현장용 `bounded_field_recovery.xml`은 후방 costmap 충돌 검사를 거친 8cm 후진 후 다시 계획한다.
후방까지 막히면 후진도 실패하며 무조건 밀고 나가지 않는다. 실패 시 구동 lease를 `none`으로
전환하고 기존 `navigate.failed`의 `error_code`로 보고한다. 현재 좌표가 불명확하면 완료로 보내지 않는다.
양쪽을 최신 코드로 설치한 뒤 runner에 `ROPORT_NAV_DIAGNOSTICS=1`을 설정하면 기존 선택 필드
`navigation`에 현재/목표 좌표, 복구 횟수, Nav2 결과와 실패 원인이 함께 온다.

관제는 실패한 단계·목표·실행 ID와 점유 상태를 HOLD에 보존한다. **자동 재출발은 하지 않는다.**
재시도 가능한 Nav2/마커 유실 오류에 한해 현장 담당자가 로봇 정지·위치·적재·통로 상태를 대조한 뒤
관제 PC에서 다음 명령을 명시적으로 실행한다. 웹의 일반 HOLD 해제 버튼은 추가하지 않았다.

```bash
uv run python tools/retry_pinky_navigation.py --socket /path/to/trial.sock \
  --execution-id FAILED_EXECUTION_ID --field-confirmed
```

터미널로 실행한 관제 owner에는 `retry FAILED_EXECUTION_ID reconciled`를 입력할 수도 있다.
같은 프로세스의 boot·binding·지도와 동일한 goal/좌표/허용오차를 유지하고, 새 command/execution ID로
기존 `navigate.requested`를 보낸다. 과거 이벤트·ACK·실패 기록은 삭제하지 않는다.
Pinky는 취소한 Nav2 작업의 **최종 종료**, 신선한 센서·위치, 실제 정지, 단독 구동권까지 확인한다.
이 조건을 만족하지 않거나 변경된 목적지가 오면 재시도를 거절한다. 같은 명령 재전송은 재실행하지 않는다.
관제/로봇 재시작, MQTT/guard/lease 장애, 결과 미확인 timeout, 지도/boot 변경은 이 절차로 해제할 수 없다.
이때는 기존 현장 대조·새 시험 준비 절차를 따른다. 재시도 명령도 실패하면 다시 HOLD로 보고한다.

### 선택 기능: 게이트 ArUco 최종 주차

별도 `dock.requested`를 시험 프로토콜에 섞지 않고, 기존 `navigate.requested`의 완료 직전에
게이트 마커를 확인한다. 관제의 pose는 **최종 로봇 기준점**, 허용오차는 최종 기준점의 오차이며
벽과의 간격이 아니다. 마커 크기·카메라 내부 보정·광학 프레임 TF·로봇 앞면 길이를 실측한 뒤,
runner와 trial launch 양쪽에 같은 `ROPORT_GATE_DOCK_CONFIG=/private/gate-docking.yaml`을 설정한다.
검증 전에는 설정하지 않는다. 아래 수치는 형식 예시이며 현장 보정값이 아니다.

```yaml
version: 1
field_verified: false
front_extent_m: 0.06
clearance_m: 0.05
image_topic: camera/image_raw/compressed
camera_info_topic: camera/camera_info
goals:
  G2: {marker_id: 9, marker_size_m: 0.10}
```

실측 후에만 `field_verified: true`로 바꾼다. goals 키는 관제가 보내는 goal_id와 정확히 같아야 한다.
기존 30cm/25cm 정차 좌표를 그대로 보내면 목표 불일치로 실패할 수 있으므로 관제의 최종 좌표와
버전도 함께 갱신한다. 도킹은 최종점 20cm 전에서 정지 → 지정 ID의 새 영상 3장 확인 →
정확한 끝점을 담은 FollowPath와 Nav2 저속 controller(최대 0.02m/s)로 접근 → 새 마커와 실제 정지 확인 순서다.
Smac의 같은 격자 목표 생략이 최종 주차에 영향을 주지 않도록 마지막 경로는 관측점까지 직접 구성한다.
목표 간격은 **앞면에서 벽까지 5cm**이며 접촉 주차를 뜻하지 않는다. 최종 위치 1.5cm/각도0.08rad
이내이고 관제 원래 목표 오차도 만족해야 completed가 된다.

잘못된 ID, 미보정 CameraInfo, 영상/TF 유실, 마커 위치 급변, 25초 초과, 관제 목표와 불일치는
`gate_dock_*` 실패로 정지한다. 영상의 타임스탬프에 맞는 map TF만 쓰며 추정 거리나 목표 좌표를
관측값으로 대신하지 않는다. 도킹도 Nav2 충돌 검사·기존 guard/속도 watchdog을 통과한다.
실제 근접 거리에서 마커가 카메라 시야에 남는지는 별도로 실물 확인해야 한다. 현재는 합성 영상과
ROS 모의 action 검사이며, 실물 마커 주차 검증 완료를 뜻하지 않는다.
