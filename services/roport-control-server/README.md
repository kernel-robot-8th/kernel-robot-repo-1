# roport-control-server

RO:PORT · GATE CREW — 통합관제 및 로봇 통신

## 기술 스택

Python 3.12 / FastAPI / uv / Three.js / Paho MQTT / pytest / Playwright / Ruff. 정확한 버전은 manifest와 lockfile을 따릅니다.
관제 서버는 AI 노트북에 배치하고 다른 PC의 브라우저에서 접속합니다. 실행부는 별도 프로세스 1개이며 서비스 WSS와 로봇 MQTT를 담당합니다. 운영 DB는 상태·요청·이력을 저장하고, API와 실행부의 조작 연결은 Unix socket을 사용합니다. AI 추론은 API와 별도 프로세스로 운영합니다.
현재 구현은 **웹 UI ↔ FastAPI ↔ Unix socket ↔ Fleet 실행부 ↔ MQTT v2**이며 OFFLINE/DEMO/TRIAL 모드도 제공합니다. Fleet 실행 스냅샷·서비스 요청은 PostgreSQL/WSS로 연결하며 운영 배포·카메라 웹 중계는 후속입니다. 공유 등록은 Fleet의 `--registry` 모드에서 같은 API/UI와 PostgreSQL에 연결하며, Pinky 시험은 **원본 웹 UI ↔ FastAPI ↔ Unix socket ↔ 시험 실행부 1개 ↔ MQTT**로 연결합니다. 시험 이력은 파일 journal에 저장합니다. 브라우저는 DB/MQTT/UDP에 직접 연결하지 않습니다.

## 공통 실행부 · 구간별 시험

공통 `two_pinky` 설정으로 `--smartcart-only` 없이 한 번 실행하면 원본 UI의 **시나리오 선택 · 노드 편집**에서 1대·2대·스마트카트 기본안을 불러오고 노드 경로를 편집할 수 있습니다. 선택한 계획의 참여 기체만 연결을 요구합니다. 같은 패널에서 라인 이동·작업 완료 확인·물품 요청 대기·N12 모드 전환·Nav2 목표 이동을 조합합니다. 기존 1대/2대/스마트카트 시험도 같은 실행부의 기본 계획입니다. 시험 출고 요청은 현재 위치에 따라 기체와 최소 간선 경로를 선택하며, 이 순차 시험과 별도로 Fleet은 서비스 WSS·재고·자원 예약에 따른 동시 주행을 구현합니다.

설정 검증 → 서버 저장(선택) → 실행 계획 적용 → 별도 출발 순서입니다. 실행 중 편집 적용과 HOLD 재개는 금지하며, 완료 후 새 실행은 이전 결과를 자동 보관합니다. 기존 Pinky MQTT schema/runner/ROS 드라이버와 `trial.json`은 그대로 사용합니다. 현재 주행기의 경로 종료점은 정차선 노드만 가능하며 교차로는 경유 지점입니다. 자세한 범위·명령·제약은 [주행 시험 가이드 7.7](docs/pinky-trial.md#77-공통-실행부와-구간작업-설정)을 따릅니다.

## 구조 · FFT1-122

```
src/roport_control_server/
  domain/      __init__.py만: Workflow·Allocation·State/Recovery 규칙 자리 (골격, 미구현)
  executor/    fleet.py·fleet_traffic.py·fleet_runtime.py: 배차·통행·MQTT/서비스 실행
               fleet_socket.py·fleet_recovery.py: 웹 조작·복구; plans.py·pinky_trial.py: 순차 주행 시험
  contracts/   snapshot.py·robots.py: 표시·등록 계약과 검증 (표준 라이브러리만)
  adapters/
    mqtt/      MQTT 5 전송·field v2 명령/상태/저장 ACK, 기존 trial-v1 시험 계약
    db/        공유 등록·Fleet 스냅샷·서비스 inbox/outbox
    wss/       서비스 서버 연결·저장 ACK·재전송
    udp/       udp_frame_protocol.py·udp_frame_sender.py·udp_frame_receiver.py: UDP 영상 wire v1
    api/       app.py: HTTP/WebSocket, display.py: 모의 표시, maps.py: 읽기 전용 지도
  spatial_assets/  index.html·scene.js·style.css 및 고정 모델/Three.js 자산
scripts/       mqtt_demo.py·udp_video_demo.py: 모의 예제 (wheel 미포함, MockExecutor 포함)
tests/         test_layers.py: 아래 import 규칙 검사
```

`spatial_assets/index.html`·`scene.js`·`style.css`가 브라우저 표시 계층이며 `adapters/api/app.py`가 정적 파일과 REST/WebSocket을 제공합니다. 기존 `adapters/ui`·Qt 표시 계층은 제거됐습니다.

| 위치 | 허용 import |
| --- | --- |
| `contracts/` | 표준 라이브러리 |
| `adapters/mqtt/` | 표준 라이브러리, paho, 자기 폴더 |
| `adapters/udp/` | 표준 라이브러리, 자기 폴더 |
| `adapters/api/` | 표준 라이브러리, fastapi, Pillow, PyYAML, `contracts`, 자기 폴더 |
| `domain/` (골격) | 표준 라이브러리 |
| `executor/` | 표준 라이브러리, `domain`, `contracts`, `adapters`(api 제외) |

패키지는 `scripts`·`tests`를 import하지 않고, DB 드라이버(psycopg·sqlite3 등)는 `adapters/db/` 밖에서 import하지 않습니다. `domain/`은 골격이며 `executor/pinky_trial.py`는 웹 API와 IPC로 연결되는 DB 비의존 시험 실행부입니다. `adapters/db/`는 등록 저장소와 Fleet 서비스 저장소를 제공하고, `adapters/wss/`는 명시적으로 설정한 Fleet 실행부에서 연결합니다. API는 Unix socket으로 Fleet을 호출합니다. 규칙이 없는 새 폴더는 `tests/test_layers.py`가 실패시키므로 규칙을 함께 추가합니다.

## 현재 범위

MQTT 5 작업 명령·이벤트 모듈과 모의 Pinky·OMX 왕복 예제, FastAPI에서 제공하는 3D 웹 관제 화면을 제공합니다.
UDP 영상 송수신 모듈(FFT1-105)도 포함합니다. 공유 등록·설정 요청·이력 저장소를 제공합니다. Fleet API/UI는 구현됐고 공유 등록은 `--registry`로 선택합니다. 실제 기체 설정 적용 검증·운영 인증·배포는 후속입니다. 업무 CRUD는 만들지 않습니다.
CI 성공은 실로봇·GPU·통합 검증 완료를 의미하지 않습니다.
배포 환경 미정으로 CD와 자동 배포는 설정하지 않았습니다.

## Fleet MQTT 실행과 도착 재확인

`executor.fleet_runtime`은 루프백 MQTT를 사용하며 시험용 journal 또는 PostgreSQL 중 하나를 명시합니다. 기본 실행은 simulation이며, 현장은 검증된 설정과 별도 구동 승인을 사용하는 현장 실행 도구를 따릅니다. 로컬 소켓의 `enable`로 작업 처리를 시작합니다.

```bash
uv run python -m roport_control_server.executor.fleet_runtime \
  --config /tmp/fleet.json --journal /tmp/fleet-control.json --socket /tmp/fleet.sock
```

기체가 도착 확인 전에 재부팅하면 로컬 소켓의 `snapshot`에 있는 `pending_arrivals`에서 해당 기체의 `node`, `command_id`, `recovery_boot`을 확인합니다. 해당 노드에서 기체와 정차 상태를 확인한 후 실행합니다.

```bash
python3 tools/confirm_fleet_arrival.py --socket /tmp/fleet.sock \
  --robot pinky1 --command-id "$ARRIVAL_COMMAND" --boot-version "$RECOVERY_BOOT" \
  --note "도착 노드에서 기체 정차 확인"
```

실행부가 새 기동의 최신 정차 상태와 도착 노드를 검증하고 확인 기록을 남깁니다. Fleet이 HOLD이면 별도로 `release_hold`와 `enable`을 실행해야 합니다.

Fleet은 발행이 확인되지 않은 STOP을 같은 명령 ID로 유효기간 안에 재시도합니다. STOP이 만료됐거나 기체가 재부팅했으면 운영자가 최신 상태를 확인하고 로컬 소켓에서 `recover_stop`을 요청합니다. 요청에는 현재 `session_id`, `expires_at`, 작업 명령의 `command_id`, 현재 기록에 남은 `stop_id`, 로봇의 현재 `boot_version`, 확인한 내용을 적은 `note`가 필요합니다. 같은 기동에서 원래 명령이 여전히 활성 상태면 새 STOP을 발행합니다. 기체가 정차·유휴 상태면 그 확인만 기록하며, 물품 제거와 작업 종료는 별도 `reconcile`로 처리합니다.

주행 중 재부팅한 Pinky는 기존 journal의 `active_command_id`와 `fault=recovery_required`를 유지합니다. 이 경우 `confirm_arrival`로 도착을 만들지 않고 다음 순서로 작업을 정리합니다.

1. Fleet HOLD와 기체 정지를 확인합니다. `snapshot`의 현재 기동 ID·기존 활성 명령 ID·STOP ID를 사용해 `recover_stop`을 요청합니다. 새 기동에서는 최신 정차·라인 비활성·제어권 해제와 `recovery_required`가 모두 확인돼야 새 STOP을 발행합니다.
2. 기존 주행의 `interrupted`와 새 STOP의 `completed / stopped=true`가 저장될 때까지 기다립니다. 기체 `active_command_id`는 없어지고 `fault=operator_stop`은 유지됩니다. 이때 작업·자원 예약은 해제되지 않습니다.
3. 운영자가 물품을 제거하고 해당 역할의 대기점에 기체를 배치한 뒤, **같은 journal 경로**로 Pinky MQTT 실행부와 드라이버를 재시작합니다. 실제 배치에 맞는 시작 노드·방향 및 정차·유휴·장애 없음이 보고돼야 합니다. journal 삭제나 재고 직접 수정으로 복구하지 않습니다.
4. 로컬 소켓에 `reconcile`(`command_id`, `outcome="removed"`, `note`)을 요청해 해당 작업을 CANCELED로 정리합니다. 기존 주행을 완료나 고객 수령으로 처리하지 않습니다. 다른 보류 작업도 정리한 뒤 `release_hold`(`note`), 별도 `enable` 순서로 재개합니다. 모든 조작에는 현재 `session_id`와 유효한 `expires_at`이 필요합니다.

이 절차는 실제 MQTT endpoint·가상 드라이버·루프백 브로커·운영자 소켓으로 검증하며, 실물 재배치·정차 확인은 현장 검증 범위입니다. Pinky 메시지 규격과 endpoint 코드는 변경하지 않습니다.

STOP 수락 직후 **다시 재부팅**하면 새 기동에서 생성된 STOP 완료와 원래 주행의 중단 응답은 이전 기동을 대상으로 한 명령과 맞지 않아 거절됩니다. 위 2단계에서 완료 응답을 계속 기다리지 말고, HOLD를 유지한 채 3단계의 물품 제거·대기점 배치·같은 journal 재시작을 수행합니다. 이후 최신 정차·유휴 상태의 `boot_version`으로 기존 작업 `command_id`와 현재 `stop_id`를 지정해 **`recover_stop`을 다시 요청**하고 `reconcile(outcome="removed")`를 진행합니다. 이때 재확인은 정지 상태에 대한 운영자 기록이며, 거절된 응답을 완료 근거로 바꾸지 않습니다. HOLD 해제와 별도 `enable`은 여전히 필요합니다.

기동이 맞지 않는 두 응답은 관제 `events`에 저장하거나 애플리케이션 `event_stored` ACK를 보내지 않습니다. MQTT 전달 ACK와 구분하며, Pinky `records`·`outbox`에 증거로 보존되어 실행 중에는 재전송될 수 있습니다. 취소가 확정된 뒤 다시 수신해도 작업·재고를 진행시키지 않습니다. 이 PR은 별도 이벤트 정리 기능을 추가하지 않습니다. 시험 종료 후 양쪽 journal을 함께 보관하고, 미정리 작업이 있는 journal이나 outbox를 직접 지우지 않습니다.

상태·위치 관측이 허용 시각 범위를 벗어나면 `clock_skew_or_delay`와 기체·메시지 종류·관측 경과 시간을 로그에 남깁니다. 일반 관측은 3초이며 실행 중 선반 작업의 관측·유지 확인은 8초 기준입니다. 시계 동기화와 전달 지연을 점검하며 현장 오류를 숨기려고 기준을 임의 변경하지 않습니다. 예상 밖 실행 오류에서도 종료 전에 HOLD를 저장하고 자원을 닫으며, 오류 자체는 숨기지 않습니다. `fleet_profiles(simulation=False)`에는 모의 N12 복귀점·허용 오차·IR 오프셋을 담은 `n12_goal`을 제공하지 않습니다. 실물용 측정·합의 후 따로 설정해야 하며 루프백 브로커 제한과 현장 구동 승인은 유지합니다.


## Fleet 웹 UI 연결

기존 3D 화면을 `?mode=fleet`으로 열어 단일 Fleet 실행부의 상태·역할·선반·입출고 요청을 공유합니다. 웹 서버는 MQTT나 journal을 직접 쓰지 않고 Unix 소켓으로 조작하며, 화면 접속·새로고침만으로 작업 처리를 시작하지 않습니다. 기존 TRIAL/DEMO/OFFLINE 모드는 유지합니다.

앞 절의 Fleet 실행부와 **같은 사용자 계정·PC**에서 다음을 실행합니다. 지도와 layout은 Fleet 설정을 생성할 때 사용한 파일과 같아야 합니다.

```bash
export ROPORT_MAP="$PWD/../roport-pinky/maps/airport_map.yaml"
export ROPORT_TRIAL_LAYOUT="$PWD/../roport-pinky/src/roport_gz_world/layout/roport_airport.yaml"
export ROPORT_FLEET_SOCKET=/tmp/fleet.sock
export ROPORT_TRIAL_TOKEN="$(python3 -c 'import secrets; print(secrets.token_urlsafe(32))')"
uv run uvicorn roport_control_server.adapters.api.app:app --host 127.0.0.1 --port 8000
```

`http://127.0.0.1:8000/ui/?mode=fleet`에서 `ROPORT_TRIAL_TOKEN` 값을 조작 키에 입력합니다. 역할·입출고 요청을 지정하고 **요청 처리 시작**을 누릅니다. 조작 키는 브라우저 저장소에 보관하지 않아 새로고침 후 다시 입력합니다. 실제 고객 수령 증거가 없는 수동 수령 버튼은 simulation 모드에만 표시합니다.

- 상태: 현재 관측 좌표·노드, 상태 지연, 역할, 6칸 선반, 작업 단계, 미완료 명령과 자원 예약을 표시합니다. 마지막 노드 위치는 흐리게 표시하는 참고 위치이며 도착 근거로 사용하지 않습니다.
- 중단: **관제 보류**는 실행부의 기존 STOP/HOLD 처리를 요청합니다. 현재 실행부 세션에 보내는 로컬 STOP 요청은 시간 만료로 버리지 않습니다. 로컬/HTTPS에서는 상태 조회를 먼저 기다리지 않고 STOP을 전달합니다. 응답 시간 초과는 정지 실패·성공을 확정하지 못하므로 현장 정지 수단과 기체 상태를 확인합니다. MQTT로 기체에 전달하는 STOP의 유효기간과 완료 확인 조건은 유지합니다. 브라우저 연결이 끊기면 조작을 막고 마지막 수신 상태임을 표시합니다. 물리 비상 정지를 대체하지 않습니다.
- 요청 재시도: 입출고 요청은 거절·응답 미확인 시 같은 `request_id`를 유지합니다. 접수 성공 또는 입력 변경 후에는 새 ID를 사용합니다. JSON이 아닌 오류 응답도 HTTP 상태와 함께 안내합니다.
- 복구: **중단·복구 확인**에서 대상과 현장 확인 기록을 입력합니다. 저장된 완료 반영·거절 작업 재배정·물품 제거 후 취소는 기존 실행부의 증거 검사를 그대로 통과해야 합니다. STOP 재요청과 재부팅 후 도착 재확인은 별도 버튼이며, 화면이 보고 있던 명령·기동 ID를 전달합니다. 대상/처리 변경이나 연결 단절 시 확인 체크를 다시 해야 합니다.
- 재개: 미완료 명령과 도착 재확인을 처리한 뒤 **보류 해제**, 별도 **요청 처리 시작** 순서입니다. 조회나 복구 성공을 자동 재출발로 취급하지 않습니다.

위 예시는 루프백 simulation Fleet용이며, Fleet 자체는 서비스 연결과 PostgreSQL을 포함합니다. 두 대의 현장 Fleet에 연결하는 시험은 [서비스 실행 절차](docs/service-web-local-test.md#서비스-요청을-두-대의-현장-fleet에-연결하기)를 따릅니다. 운영 인증은 후속입니다. 웹 화면/가상 실행기 시험은 실제 센서·팔·주행 검증을 대신하지 않습니다. 전체 가상 실행은 `tools/start_virtual_ui.sh`, 고객 서비스 통합은 [실행 안내](docs/service-web-local-test.md)를 따릅니다. UI 회귀는 `npm run test:web -- tests/browser/fleet.spec.mjs`, HTTP→MQTT 입출고 회귀는 `REQUIRE_MQTT_TESTS=1 uv run pytest tests/test_fleet_web_mqtt.py`로 실행합니다.

### Fleet 회귀시험과 가상 물품 선택

```bash
# 기본 실행은 요청 대기. 자동 입고·출고·수령·복귀는 명시적으로 선택한다.
bash tools/start_virtual_ui.sh --verify        # 소형: 기존 옵션 유지
bash tools/start_virtual_ui.sh --verify large  # 대형: OMX1·2 공동 상하차
# 자동 시험은 완료 후 종료하며 실행 폴더의 outcome.json에 case·tasks를 남긴다.

# Mosquitto와 격리 PostgreSQL(ROPORT_TEST_DATABASE_URL)이 필요한 회귀시험
REQUIRE_MQTT_TESTS=1 REQUIRE_REGISTRY_TESTS=1 uv run pytest \
  tests/test_fleet_concurrent_requests.py tests/test_fleet_reassignment_mqtt.py \
  tests/test_fleet_spatial_matrix.py tests/test_fleet_persistence.py \
  tests/test_virtual_fleet_launcher.py
```

| 검증 | 범위 |
|---|---|
| 동시 요청·복귀 재배차 | 실제 loopback MQTT와 Pinky 실행 코드로 중복 배정·경로 점유·병목 해제·수령 대기를 검사 |
| 공간 조합 | 현재 지도, 대기점 N16/N17·H1/H2, 선반 6칸, 제외 기체·막힌 노드·가득 찬 선반을 합성 상태로 검사 |
| 저장·재시작 | 임시 DB 저장 실패 시 명령 발행/완료 ACK를 막고, 재시작 후 작업·점유 보존 및 자동 출발 금지를 검사 |

MQTT 회귀는 센서 tick을 가속한다. 가상 실행기는 1배속이며 OMX 구동·Nav2·센서는 모사한다.
통신 단절·중복 응답·기체 재시작·한쪽 팔 실패는 기존 `test_fleet_mqtt.py` 및 정지·복구 시험과 함께 전체 `uv run pytest`로 검증한다. 물리 충돌·제동 거리·무선 품질 검증은 포함하지 않는다.

## Pinky MQTT 주행 시험 · FFT1-56

1대/2대/스마트카트 시나리오를 CLI 또는 원본 웹 UI에서 실행합니다. `route → 수동/MQTT 상하차 완료 → N12 mode_switch → navigate`를 완료 응답으로 진행하며, 파일 이력·중복 방지·저장 ACK·오류 시 HOLD를 포함합니다. 같은 실행부를 사용하며 운영 DB는 연결하지 않습니다. 웹 연결·조작 키·시험 범위는 [가이드 7.4절](docs/pinky-trial.md)을 참고하세요.

**팀원 재현 순서와 메시지/이식 방법은 [주행 시험 가이드](docs/pinky-trial.md)를 따르세요.** `tools/prepare_pinky_trial.py`로 네 설정을 한 번 생성하고 같은 `trial.json`을 양쪽에 배포합니다. `tools/build_pinky_mqtt_kit.py`는 Paho만 필요한 이식 ZIP·JSON Schema·19개 메시지 예제를 생성합니다.

실제 ROS 드라이버와 독립 정지 감시는 [Pinky PR #35](https://github.com/kernel-robot-8th/kernel-robot-repo-1/pull/35)가 담당합니다. 가상 시험은 이 관제 브랜치만으로 재현할 수 있습니다. 실물은 해당 핑키 코드와 새 이식 키트를 함께 사용하고 현장 방향·골·정지 검증을 완료해야 합니다. 생성한 hardware 설정은 기본 `field_verified=false`라서 구동을 허용하지 않습니다.

## 공유 등록 저장소

`contracts/robots.py`는 요청 검증과 DTO, `adapters/db/registry.py`는 PostgreSQL 트랜잭션을 담당합니다. Fleet의 `--registry` 모드는 API → Unix socket → 단일 owner를 통해 등록과 역할을 함께 저장합니다. `RegistryStore.snapshot()`, 개발용 `change(robot_id, action, data, development=...)`, `history(robot_id, limit=...)`도 유지합니다. DB 드라이버는 저장소 안에만 둡니다.

- Pinky 고정 3자리와 OMX 조회용 3자리를 생성합니다. OMX1·2는 같은 Host에 서로 다른 arm ID로 표시하며 OMX 등록 변경은 제공하지 않습니다.
- 등록·요청 설정 변경·운영 제외·해제를 저장합니다. 활성 기체 ID/IP/ROS 도메인 중복, 이전 binding·낡은 revision을 거절하며 같은 request ID의 동일 요청은 기존 결과를 반환합니다. 변경·revision·이력·재시도 결과는 한 트랜잭션입니다.
- 요청 설정(`desired_config`)을 저장하며 실제 적용을 추정하지 않습니다. 단독 Registry 조회는 관측 unknown·준비 false이고, 연결된 Fleet은 MQTT의 정차·준비 상태를 함께 표시합니다. 기체 파일·runtime 설정 일치와 적재 여부는 계속 unknown입니다.
- 기존 개발용 쓰기는 개발 DB·`development=True`·`sim-` 기체로 제한합니다. Fleet owner의 등록 연결은 최신 정차·진행 작업/명령/자원 없음 조건을 사용하며 등록 정보와 Fleet 역할을 같은 DB 트랜잭션에 저장합니다.
- 배차 제외는 기체의 지도 점유를 유지합니다. 등록 해제는 먼저 제외한 뒤 운영자가 빈 적재·통로 밖 배치를 확인해야 합니다. 확인값은 MQTT 센서 관측이 아닌 운영자 입력으로 기록합니다. 해제 전 기체의 부팅·도착·위치를 새 등록에 재사용하지 않습니다.
- 연결/스키마 실패는 `RegistryError`로 반환하고 빈 목록이나 메모리 저장소로 대체하지 않습니다. 저장소 오류는 충돌 409·개발/실기 제한 403·저장소 불가 503으로 구분하며 DB 접속값은 포함하지 않습니다. Fleet 명령 API는 기존 경계에 따라 owner 거절을 HTTP 409와 오류 이유로 반환합니다. 재연결한 store도 같은 상태·이력을 읽습니다.

FastAPI는 DB에 직접 연결하지 않으며 Fleet owner는 명시된 DB에 연결합니다. 서버 시작으로 스키마를 자동 초기화하지 않습니다. PostgreSQL 16의 **비어 있는 전용 개발 DB**를 별도로 준비하고 다음을 한 번 실행합니다. `ROPORT_DATABASE_URL`은 표준 PostgreSQL DSN 환경변수이며 아래 예제는 loopback의 개인 개발 DB용입니다. 실제 인증정보는 저장소에 넣지 않습니다.

```bash
export ROPORT_DATABASE_URL='host=127.0.0.1 dbname=roport_registry_dev'
uv run python -m roport_control_server.adapters.db.migrate --development
uv run python -m roport_control_server.adapters.db.service_tasks
```

초기화는 트랜잭션·advisory lock으로 직렬화하며 반복 실행은 기존 등록을 보존합니다. 기존 DB의 개발/운영 모드 변경은 거절합니다. 스키마 1 초기화만 제공하며 업그레이드·자동 rollback·데이터 삭제 명령은 없습니다. 운영 적용과 백업/복구 절차는 별도 검증해야 합니다.

```bash
export ROPORT_TEST_DATABASE_URL='host=127.0.0.1 dbname=roport_registry_test'
REQUIRE_REGISTRY_TESTS=1 REQUIRE_MQTT_TESTS=1 uv run pytest
```

테스트 DSN에는 운영 DB를 지정하지 않습니다. 테스트마다 UUID 스키마를 만들고 자신이 만든 스키마만 삭제하며, 접속 계정에는 그 권한이 필요합니다. CI는 PostgreSQL 16과 필수 플래그로 DB 테스트 누락을 실패 처리합니다. `ROPORT_TEST_DATABASE_URL` 없이 일반 pytest를 실행하면 DB 통합 검증은 skip됩니다.

기존 owner 실행 명령에 `--registry`를 추가하면 공유 등록을 사용합니다. 사전 생성한 PostgreSQL Registry가 필요하며, 최초 전환은 미완료 작업·명령·예약이 없는 상태에서만 가능합니다. 미등록·제외 슬롯에는 역할을 배정하지 않습니다. 공유 등록을 사용하지 않은 DB/Journal은 옵션 없이 기존 프로필 기반 역할 처리를 유지합니다. 이미 Registry를 연결한 DB는 역할 불일치를 막기 위해 재시작에도 `--registry`가 필요합니다. 정비 SSH·로봇 설정 배포는 연결하지 않습니다.

새 격리 DB에서 UI 등록을 시험하려면 `./tools/run-latest-service-test.sh --registry`를 실행합니다. 관제 조작 키를 입력하고 Pinky 탭에서 `sim-` 기체 ID와 역할을 등록한 뒤 별도로 요청 처리를 시작합니다. 기존 시험 DB나 운영 DB를 초기화하지 않습니다.

## 3D 웹 통합관제 · FFT1-109

PR #24 계층 구조 위에서 기존 3D HTML/CSS/Three.js를 재사용하고 Qt/QWebChannel을 HTTP/WebSocket으로 교체했습니다. Qt·데스크톱·Node.js 런타임 없이 설치 wheel만으로 서버를 실행할 수 있습니다. Node.js는 소스 자산 준비와 브라우저 테스트에만 필요합니다.

```bash
npm ci --ignore-scripts
npm run prepare:assets
uv sync --locked
uv run uvicorn roport_control_server.adapters.api.app:app --host 127.0.0.1 --port 8000 --ws-max-size 4096 --ws-max-queue 4
```

- OFFLINE: `http://localhost:8000/ui/?mode=offline` — 실행부 미연결, 작업 없음, 실제 위치 미표시.
- TRIAL: `ROPORT_TRIAL_SOCKET`을 설정하면 `/ui/`가 시험 모드로 연결됩니다. 미설정 시 기본 OFFLINE입니다. 지도·레이아웃·24자 이상 조작 키가 필요하며, API 시작이나 화면 접속만으로 출발하지 않습니다.
- DEMO: `http://localhost:8000/ui/?mode=demo` — 예시 Pinky 3대/OMX 3대의 모의 동작. 재생·시각 이동·수신 중단은 이 브라우저 세션에만 적용됩니다. 재접속 시 DEMO는 새 세션으로 시작하며 실제 작업 복구 시험이 아닙니다.
- 서버가 제공한 목록으로 화면을 만들지만 DEMO 목록은 6대 예시이며 FLEET 화면은 실행부의 상태를 표시합니다. `--registry` 모드의 Pinky 설정은 고정 세 슬롯의 공유 등록을 표시합니다. 등록 정보만으로 실제 기체 설정 적용을 확인했다고 표시하지 않습니다.
- Pinky 탭 > Pinky Settings(FFT1-124, 시안 v0.2): 등록(이름·IPv4·역할·ROS_DOMAIN_ID 0~101·IR 기준값)과 작업(연결 확인·도메인 읽기/적용·업데이트/릴리즈 설치·빌드) 화면입니다. `contracts/pinky_settings.py`가 입력 형식을 정합니다.
  OFFLINE에서는 `POST /api/pinky/settings/robots`·`/jobs`가 형식만 검사하고 `503 executor_not_connected`를 돌려줍니다(저장·실행 없음). DEMO는 예시 3대를 브라우저 안에서만 바꿉니다.
  FLEET의 `--registry` 모드는 기존 `/api/fleet/commands`의 조작 키·출처·세션 검사를 거쳐 등록/수정/제외/해제를 저장합니다. 설치·빌드·도메인 적용은 비활성 상태를 유지하며 로봇 설정 파일을 수정하지 않습니다.
- 기존 회백색 3열 구성, 3D/평면·회전·선택 장비 확대, 관절 모델과 작업·오류 표시를 제공합니다. 실기 URDF 보정과 양팔 협업 동기화는 검증하지 않았습니다. OFFLINE/DEMO의 승인·출발·취소는 비활성화합니다. TRIAL에서는 시험 실행부가 허용하는 출발·수동 완료·물품 요청·보류만 조작합니다.

### 접속·지도·운영

현장 LAN의 다른 PC에서도 보려면 위 명령의 `--host`를 `0.0.0.0`으로 바꾸고 브라우저에서 `http://<서버-LAN-IP>:8000/ui/`를 엽니다. 방화벽은 허용된 현장 망으로 제한합니다. 실물 시험의 원격 조작은 HTTPS로만 허용하고 HTTP LAN 접속은 조회만 가능합니다. OFFLINE/DEMO 조회는 로그인 인증이 없고 TRIAL 조작 키도 운영 계정/권한 체계를 대체하지 않으며 인터넷 공개·운영 데이터 연결 전 인증/권한·HTTPS/WSS 또는 VPN이 필요합니다. FastAPI와 화면을 같은 origin으로 제공해 별도 CORS 허용이 필요 없습니다.

상태 WebSocket은 **worker당 OFFLINE/DEMO/TRIAL 합계 8개**로 제한합니다. 초과 접속은 handshake 전에 거절되므로 실제 HTTP 응답은 403이며 브라우저는 기존 재접속 정책을 따릅니다. 연결 종료·오류 시 송수신 작업을 정리하고 자리를 반환합니다. 현재는 worker 1개로 운영하며, 8개는 AI 병행 부하를 보장하는 측정값이 아닌 임시 상한입니다. HTTP 요청·IP별 제한은 포함하지 않습니다.

TLS 역방향 프록시는 WebSocket upgrade, 브라우저의 `Origin`, 외부 `Host`(비기본 포트 포함)를 그대로 전달하고 `X-Forwarded-Proto`를 실제 외부 scheme으로 설정해야 합니다. 같은 서버의 loopback 프록시라면 위 Uvicorn 명령에 `--proxy-headers --forwarded-allow-ips=127.0.0.1`을 사용합니다. 원격 프록시는 실제 프록시 IP만 신뢰하고 `*`로 열지 않습니다. 신뢰된 HTTPS 전달 헤더는 Uvicorn이 `wss`로 변환하므로 같은 origin 연결이 허용됩니다. 내부 Host로 바꾸거나 신뢰하지 않는 경로에서 HTTPS 헤더만 보내면 거절됩니다. `X-Forwarded-Host`만으로는 보완되지 않습니다.

서버 운영자가 `ROPORT_MAP=/절대경로/map.yaml`로 PNG/PGM+YAML 지도를 선택하고, `ROPORT_MAP_LAYOUT=/절대경로/layout.yaml`로 실측 레이아웃을 함께 지정할 수 있습니다. 기본 지도는 공개 예시입니다. 파일 업로드 API는 없으며 원본을 수정하지 않습니다. 지도·레이아웃은 접속자에게 표시되므로 접근 권한을 확인하고 현장 원본은 저장소 밖에 둡니다.

```bash
ROPORT_MAP=/absolute/path/map.yaml ROPORT_MAP_LAYOUT=/absolute/path/layout.yaml uv run uvicorn roport_control_server.adapters.api.app:app --host 127.0.0.1 --port 8000 --ws-max-size 4096 --ws-max-queue 4
```

- 레이아웃은 `units: cm`, `version`, `map: {width, height, resolution, origin: [x, y]}`, `frame: {x_m, y_m, yaw_deg}`, `defaults.wall_height`, `nodes: {graph_version, items}`를 사용합니다. 노드는 `id`, `at: [x, y]`, 대칭 `links`와 선택 `place`/`nav2_switch`를 가집니다. 지도 격자가 다르거나 노드가 지도 밖이면 서버 시작을 거부합니다. 현재 실측 레이아웃은 지도 origin yaw=0을 요구하며 레이아웃 frame 회전은 지원합니다.
- 좌표는 frame의 회전·이동과 cm→m 변환 후 ROS map 좌표로 표시합니다. 노드/연결선·장소 별칭은 입력 레이아웃을 따르며 P4/P7의 의미나 주행 경로를 확정하지 않습니다. 지도 이미지와 레이아웃의 내용 해시는 별도로 제공합니다.
- `objects`의 `omx12_left`/`omx12_right`에 `min`, `max`, `height`(또는 `defaults.object_height`)를 지정하면 OMX1·2를 받침 앞 변(`max.y`) 중앙에 밑판이 닿도록 상호 대향 배치합니다. 현재 현장 배치의 N19 쪽 변에 해당합니다. 설치 배치는 기준값이며 실제 위치·관절 미수신 상태는 그대로 유지합니다.
- 레이아웃 미지정 시 벽 높이 0.35m는 표시 가정입니다. 실측 레이아웃의 벽 높이도 점유 셀 전체에 일괄 적용하는 표시 기준이며, 실제 구조물별 3D 형상은 아닙니다. DEMO 주행은 선택한 지도 크기에 맞춘 예시 애니메이션으로 장애물 회피·주행 승인에 쓰지 않습니다.

클램쉘 운영은 OS의 덮개 닫힘 절전 해제, 전원·냉각·유선 LAN, 서비스 자동 시작/재시작을 별도로 설정·검증합니다. 이 PR은 OS 설정을 변경하지 않습니다. AI 추론 병행 성능·현장 Wi-Fi·여러 운영자의 조작 권한·장시간 운영은 후속 검증입니다.

### API와 팀원 세부 탭

| 경로 | 동작 |
| --- | --- |
| `POST /api/trial/commands` | 동일 origin·조작 키와 현재 session/task/phase를 확인한 후 별도 실행부에 전달 |
| `GET /api/snapshot?mode=trial` · `WS /api/state?mode=trial` | 시험 상태 조회·0.25초 주기 표시; 브라우저에서 MQTT 직접 연결 없음 |
| `GET /healthz` | 프로세스 생존 확인; 장비 준비 판정 아님 |
| `GET /api/bootstrap?mode=auto/offline/demo/trial` | 지도·예시 장비 목록·표시 모드 |
| `GET /api/snapshot` | OFFLINE 전체 상태; DB 연동 자리 |
| `WS /api/state?mode=offline 또는 demo` | 전체 snapshot을 최대 10Hz 전송; session_id/revision 포함 |
| `/ui/` | 인터넷 CDN 없는 정적 웹 화면·3D 자산 |

WebSocket은 같은 origin만 허용합니다. DEMO에서만 `play(bool)`, `seek(0..24초)`, `drop(robot_id,bool)` 표시 명령을 받고 TRIAL 조작은 조작 키를 검사하는 별도 REST endpoint를 통해 실행부에 전달합니다. TRIAL WebSocket은 조회 전용입니다. 브라우저는 증가한 revision만 적용하고 단절/2.5초 이상 미갱신을 표시하며 재접속합니다. DEMO 장비의 2초 stale과 UI 단절 시간은 표시 시험용이며 장비 안전 timeout이 아닙니다. 오류·식별자는 HTML이 아닌 텍스트로 표시합니다.

`adapters/api/display.py`는 OFFLINE/DEMO 표시 전용입니다. 실제 상태 연동(FFT1-110)에서는 DB/실행부 상태 조회 경로를 별도로 두고, 이 생성기와 `play/seek/drop`을 실제 상태·명령 경로에서 호출하지 않습니다.

팀원은 `index.html`의 `#pinky-detail` / `#omx-detail` 영역과 웹 모듈로 세부 화면을 연결합니다. QWidget 연결점은 제거됐습니다. 상세 제어는 담당 팀원이 개발하며 향후 공통 API를 사용합니다. 현재 화면 코드는 `spatial_assets/`, API는 `adapters/api/`에 두고 MQTT/UDP 모듈과 분리합니다. 고객·업무 어드민용 `services/roport-web`은 별도 앱으로 유지합니다.

### 검증

```bash
uv run ruff check .
uv run ruff format --check .
env -u PYTHONPATH REQUIRE_MQTT_TESTS=1 uv run pytest
npx playwright install chromium
npm run test:web
uv build
uv run python tools/check_asset_package.py dist/*.whl
```

CI는 Mosquitto loopback, API·계층·지도·계약, Chromium OFFLINE/DEMO/TRIAL·단절/재접속 및 wheel 자산을 검사합니다. 설치형 PyQt 실행 명령과 `--extra ui`는 더 이상 사용하지 않습니다. [Jira FFT1-109](https://fp-team1.atlassian.net/browse/FFT1-109).

## 관련 문서

이 디렉토리는 단일 저장소의 구성요소입니다. 아래 명령은 이 디렉토리에서 실행합니다.

- [프로젝트 개요](https://github.com/kernel-robot-8th/kernel-robot-repo-1)
- [요구사항·설계 문서](../../docs/README.md)
- [Jira](https://fp-team1.atlassian.net/jira/projects/FFT1)

PR에 관련 Jira 키, 변경 내용, 검증 결과와 미검증 항목을 기록합니다.
실로봇 적용 전 장비·설정·캘리브레이션, 안전 정지·복구 절차를 확인합니다.

## UDP 영상 모듈 · FFT1-105
`adapters/udp/`는 OMX 카메라 호스트 → AI 수신기로 인코딩된 JPEG/PNG 프레임을 직접 보냅니다. `udp_frame_protocol.py`(상수·`FrameInfo`·헤더 pack/parse), `udp_frame_sender.py`(`fragment_frame`, `FrameSender`), `udp_frame_receiver.py`(`FrameReceiver`)는 표준 라이브러리만 사용합니다. sender·receiver는 protocol만 상대 import하며 서로를 import하지 않습니다. 카메라 캡처·인코딩·AI 추론·관제 실행부·UI 영상 표시는 구현하지 않습니다. MQTT 작업 명령/중요 결과와 별도 경로입니다.

- 입력: frame bytes, source/camera/stream, session UUID, 증가하는 frame_id, captured_ns, observation_group, codec. 같은 물품 관측의 두 카메라는 observation_group을 공유할 수 있습니다. frame_id는 세션 내 스트림별 단조 증가이며 재사용하지 않습니다.
- 시험 프로토콜 v1: network byte order, 120 B 고정 헤더 + 최대 1080 B 조각. datagram 1200 B·프레임 1 MiB·독립 JPEG/PNG만 지원합니다. RTP/H.264 호환 프로토콜은 아닙니다. 실제 영상 포맷 검사는 디코더가 담당합니다.
- 수신기는 지정한 IPv4+송신 포트, source/camera/stream, session만 받습니다. 새 세션은 담당 코드에서 확인한 뒤 수신기를 새로 구성합니다. IP 허용 목록·CRC32는 암호학적 인증이 아닙니다. **모의 LAN용이며 현장 인증·키 설정·암호화·MTU는 별도 검증 대상**입니다.
- 스트림당 미완성 프레임 1개만 유지합니다. 최신 프레임이 도착하면 이전 미완성 프레임을 폐기하고 중복·역순 조각을 처리합니다. 기본 1.5초 deadline은 최초 조각 도착 기준이고 중복으로 연장하지 않습니다. 완료/만료한 frame_id는 재조립하지 않습니다. captured_ns는 추적용이며 호스트 간 절대 지연 검사는 별도 시각 동기화가 필요합니다.
- 최대 32개 허용 스트림, 기본 inflight 16개·예약 payload 8 MiB입니다. 조각 수·크기를 검사한 뒤 할당하고 완료 시 최대 1 MiB의 출력 사본 및 Python 객체 오버헤드가 추가됩니다. `stats`에 수신 datagram·폐기 원인·완료 프레임/바이트 수가 쌓이고 `last_reassembly_seconds`에 수신 호스트 기준 마지막 재조립 시간이 기록됩니다. 트래픽이 없어도 수신 루프에서 `expire()`를 주기적으로 호출하세요.

`FrameSender`는 카메라별 FPS와 IP/UDP 헤더를 포함한 비트율을 제한합니다. 기본 8 Mbps에서 최대 1 MiB 프레임의 전송 시간은 약 1.19초이므로 기본 수신 timeout은 여유를 둔 1.5초입니다. 재조립이 끝나면 즉시 반환하며, 1.5초는 미완성 프레임의 최대 대기 시간입니다. FPS는 상한이며 큰 프레임에서 설정 FPS를 보장하지 않습니다. 비트율을 낮추거나 timeout을 줄일 때는 `(프레임 바이트 + ceil(프레임 바이트 / 1080) × 148) × 8 / bitrate`에 스케줄링·네트워크 지연 여유를 더해 설정하세요. 재전송·FEC·혼잡 제어는 없으며 9대 카메라의 허용 비트율 합은 실제 Wi-Fi 측정 후 정합니다.

### 원본과 이식본 관리

- 원본은 이 저장소의 `services/roport-control-server/src/roport_control_server/adapters/udp/`이며 통합관제 담당자가 관리합니다. 영상 경로는 OMX → AI 직접 전송으로 유지합니다.
- OMX 송신부는 `udp_frame_protocol.py`+`udp_frame_sender.py`, AI 수신부는 `udp_frame_protocol.py`+`udp_frame_receiver.py`를 같은 원본 리비전에서 수정 없이 한 폴더(패키지)에 복사합니다. 각 패키지 README에 원본 경로·전체 Git commit SHA·**파일별** SHA-256을 기록합니다. 캡처·추론 등 장비별 코드는 별도 호출부에 둡니다.
- 공통 변경은 원본 PR에서 검증한 뒤 OMX 송신부와 AI 수신부의 이식 PR을 연결합니다. 양쪽 파일 해시가 원본과 일치하고 송수신 검증을 통과한 뒤 함께 적용합니다. 사본에 독자적인 프로토콜 변경을 넣지 않습니다.
- 호환되지 않는 헤더/필드 의미 변경은 wire 버전을 올리고 송수신 양쪽을 갱신합니다. 현재 수신기는 `RPV1`만 허용하며 버전 불일치를 폐기합니다.

### 로컬 송수신 예제

두 터미널에서 같은 새 session UUID를 사용합니다. 아래 UUID는 1회 시험 예시이며 다시 실행할 때는 새 값을 쓰세요. 파일은 직접 준비한 JPEG/PNG입니다. mock AI는 수신된 바이트 수와 통계만 출력합니다.

```bash
# 터미널 1
uv run python scripts/udp_video_demo.py receive --session 673593ea-50ba-4c6d-ab47-8b8d209ee038
# 터미널 2
uv run python scripts/udp_video_demo.py send --session 673593ea-50ba-4c6d-ab47-8b8d209ee038 --file /absolute/path/frame.jpg --frames 10 --fps 10 --bitrate 8000000
```

예제는 loopback의 5301(수신)·5302(송신) 포트만 사용합니다. `--port`, `--source-port`, `--camera`, PNG용 `--codec png`를 지원합니다. 실제 적용 시 호출자가 socket과 허용 스트림/세션을 구성하고 장비별 카메라 ID를 합의해야 합니다.

검증: `uv sync --locked`, `uv run ruff check .`, `uv run ruff format --check .`, `env -u PYTHONPATH uv run pytest`, `uv build`. 단위·loopback·별도 프로세스 예제 시험이며 실제 카메라/AI·현장 처리량 측정은 포함하지 않습니다. [Jira FFT1-105](https://fp-team1.atlassian.net/browse/FFT1-105).

## 개발 방법

3D 표시 자산은 PR #22에서 분리한 패키징을 그대로 사용하며 웹 UI와 함께 배포합니다.
Three.js 원본은 Git에 커밋하지 않고 서비스 전용 npm 잠금 파일로 고정합니다. 소스 체크아웃에서 테스트·배포 패키지를 만들기 전 Node.js 24로 한 번 준비합니다.

```bash
npm ci --ignore-scripts
npm run prepare:assets
```

`prepare:assets`는 필요한 JS와 LICENSE만 로컬 `spatial_assets/vendor`에 복사하고 고정 해시를 검증합니다. 설치에는 npm 접근 또는 사전 캐시가 필요하며, 준비 후 빌드는 네트워크를 사용하지 않습니다. wheel/sdist에는 JS·모델·라이선스가 포함되어 설치한 앱의 실행 시 Node.js·npm·CDN이 필요 없습니다. 자원이 누락되거나 해시가 다르면 배포 빌드가 실패합니다. 모델 출처와 재생성 방법은 `spatial_assets/UPSTREAM.md`에 있습니다.

```bash
python3 -m pip install uv==0.13.5
uv sync --locked
uv run ruff check .
uv run pytest
uv build
```

API 실행: `uv run uvicorn roport_control_server.adapters.api.app:app --host 127.0.0.1 --port 8000`

`/healthz`는 프로세스 상태만 확인합니다. 로봇·DB·모델 준비 상태를 뜻하지 않습니다.

## MQTT 모듈과 팀원 이식

아래는 `messages.py`와 `mqtt_demo.py`를 사용하는 기존 모의 예제입니다. 실물 Fleet의 MQTT v2 메시지·개별 OMX 명령은 [현장 계약](interfaces/mqtt/README.md)과 [Pinky 이식 안내](docs/mqtt-field.md)를 따릅니다. 전송 모듈 `mqtt.py`는 두 경로가 공유합니다.

- `src/roport_control_server/adapters/mqtt/messages.py`: 공통 envelope, 토픽, 명령 검증 및 메모리 중복 기록.
- `src/roport_control_server/adapters/mqtt/mqtt.py`: Paho MQTT 5 연결, 재구독, 제한된 큐, 수동 transport ACK, TLS 설정.
- `scripts/mqtt_demo.py`: 관제 송신 및 가짜 Pinky·OMX 수신 예제(`MockExecutor`·`make_command`, 시험도 같은 정의를 사용). 실제 로봇 실행 코드가 아니며 wheel에 포함되지 않습니다.

팀원은 **adapters/mqtt 폴더 전체**를 자신의 Python 패키지에 복사하고 `paho-mqtt==2.1.0`만 추가할 수 있습니다. 내부 상대 import만 사용하며 Qt/ROS/DB/FastAPI를 불러오지 않습니다. 송수신 사용 흐름은 예제의 `run_control`/`run_device`를 참고하고 `MockExecutor`는 실제 실행부로 교체합니다.

`MqttConfig(host, client_id, port=1883)`는 로컬 Mosquitto용입니다. EMQX Cloud에서는 같은 코드를 쓰고 `host`, `port=8883`, `tls=True`, `username`, `password`를 설정합니다. 자격증명은 환경변수/배포 secret에서 읽으세요. 사설 CA가 필요하면 `ca_file`을 지정합니다. 인증서·호스트명 검증을 끄는 옵션은 없으며, 암호 인증은 TLS를 요구합니다. **Cloud 실제 접속·ACL·WAN 장애는 아직 검증하지 않았습니다.**

cmd/event와 trial-v1의 저장 확인 `/ack`를 구현합니다. telemetry·presence/LWT는 후속 범위입니다. 기체·작업대별 명시적인 토픽을 구독하며 OMX1·2는 `workcell/shelf-01` 하나를 사용합니다. 개별 `robot/omx1`, `robot/omx2` 명령은 허용하지 않습니다.

### 로컬 왕복 실행 (터미널 3개)

Mosquitto 2.x 실행 파일을 준비한 뒤 아래 순서대로 실행합니다. 별도 브로커를 loopback 전용 포트에 띄우며 기존 현장 브로커 설정은 바꾸지 않습니다.

```bash
# 1. 브로커
printf 'listener 18883 127.0.0.1\nallow_anonymous true\npersistence false\n' > /tmp/roport-test-mosquitto.conf
mosquitto -c /tmp/roport-test-mosquitto.conf

# 2. 모의 수신기 (이 패키지 디렉토리에서)
uv run python scripts/mqtt_demo.py pinky1 --port 18883

# 3. 관제 송신기
uv run python scripts/mqtt_demo.py control --device pinky1 --port 18883
```

OMX3 시험은 두 명령의 `pinky1`을 `omx3`로, OMX1·2 공동 작업대는 `shelf-01`로 바꿉니다. `accepted`와 `completed` 모의 이벤트를 확인하고 각 터미널에서 Ctrl+C로 종료합니다. 모든 예제는 `127.0.0.1`, site=`test`, 모의 허가만 사용합니다. Pinky의 `test-v1` 경로와 OMX `load` payload는 **시험용 부분 계약**이며 최종 현장 명령 계약이 아닙니다.

### 처리 순서와 한계

1. `start()`는 MQTT 연결·SUBACK까지 기다립니다. `ready`는 기체 준비나 작업 승인 상태를 의미하지 않습니다.
2. `receive()` → `decode()` → 수신자·권한·현재 작업 검증 → 실행부 처리 → `ack(delivery)` 순서로 소비합니다. 네트워크 callback은 정상 크기 메시지만 큐에 넣고 크기 초과는 경고·계수 후 transport ACK로 폐기합니다. 잘못된 메시지는 거절/기록 후 ACK하며, 정상 메시지의 영속화가 필요하면 호출자가 commit을 먼저 완료해야 합니다.
3. `publish()` 성공은 브로커 PUBACK입니다. accepted/completed/DB 반영과 구분하세요. cmd/event는 QoS1·retain=false이고 cmd에는 MQTT5 expiry와 수신 측 expires_at 검사 둘 다 적용합니다. 컴퓨터 간 시각 동기화가 필요합니다.
4. 단절은 자동 재연결·재구독합니다. 이전 연결의 큐 항목/ACK는 거절합니다. 큐 초과는 연결을 닫고 오류로 알립니다. 호출자는 상태를 재조정한 후 새 transport를 만들어야 합니다. timeout·단절 직전 발행은 전달 여부가 불명확하며 Paho가 미확인 QoS 메시지를 다시 보낼 수 있으므로 동일 command_id로 대조합니다.
5. `CommandHistory`는 단일 소비자·메모리 전용이며 가득 차면 새 명령을 거절합니다. **프로세스 재시작 중복 방지·exactly-once·DB 복구를 보장하지 않습니다.** 실로봇 이식 전 실행 착수 기록의 영속화, 재시작 재조정, 자원 허가·도킹·모델/보정 검증 및 현장 정지 처리가 필요합니다. 이 PR은 모터를 구동하지 않습니다.

데모는 `completed`를 확인해야 성공하며, 중복·이전 revision은 무시합니다. `failed`/`rejected`는 실패이고 완료 대기에는 전체 시간 제한이 있습니다. 통신 오류·timeout은 결과 불명을 알리고 종료합니다. 상태 대조 없이 데모를 재시작하면 메모리 이력이 사라지므로 자동 재실행하지 않습니다.

명령은 라우팅·retained·형식을 먼저 검사하고, 중복 기록을 조회한 뒤 **새 실행에만 만료 시간**을 적용합니다. 만료된 동일 명령의 재수신은 보관된 결과를 반환합니다. 브로커의 발행 거절은 해당 `publish()`만 실패시키며 연결은 유지합니다. 현장에서는 브로커 ACL로 명령 발행자를 제한해야 합니다.

기본 시험 한도는 메시지 64 KiB·수신/송신 큐 128개이며 운영 부하 기준은 아직 확정하지 않았습니다. 모의 공동 작업대에서는 양팔 복귀가 모두 확인돼야 completed가 됩니다. 이 문단의 모의 예제와 달리 Fleet v2는 저장·ACK·정차 근거에 따라 다음 주행을 배차합니다.

### 검증

```bash
uv sync --locked
uv run ruff check .
uv run ruff format --check .
REQUIRE_MQTT_TESTS=1 uv run pytest
uv build
```

통합 테스트는 실제 Mosquitto를 임시 loopback 포트로 실행·종료합니다. PATH에 없다면 `MOSQUITTO_BIN=/절대경로/mosquitto`를 지정합니다. 브로커가 없을 때 일반 pytest는 통합 시험을 skip하지만 CI와 위 명령은 실패하므로 누락을 숨기지 않습니다. ROS가 로드된 터미널에서 pytest 플러그인 충돌이 나면 `env -u PYTHONPATH REQUIRE_MQTT_TESTS=1 uv run pytest`로 관제 가상환경만 사용하세요.

관련 작업: [FFT1-104](https://fp-team1.atlassian.net/browse/FFT1-104). 설계 참고: [MQTT](https://fp-team1.atlassian.net/wiki/spaces/MFS/pages/10125435), [Pinky 인터페이스 검토안](https://fp-team1.atlassian.net/wiki/spaces/MFS/pages/2752522). 후자의 v0.1은 검토 중이며 이 예제가 현장 계약 확정을 대신하지 않습니다.

## 실시간 위치·가상 Pinky 시험

TRIAL 화면은 표시용 MQTT motion의 출처·관측 시각·지도 버전을 확인해 위치를 갱신합니다. 주행 명령/완료 판정과는 별도 경로이며, 지연·단절 시 마지막 위치를 표시합니다. 실물 Pinky에도 이 PR의 MQTT 모듈과 `trial_driver.py`를 함께 적용해야 연속 위치가 발행됩니다.

```bash
# 이 서비스 폴더에서. PATH에 Mosquitto가 없으면 MOSQUITTO_BIN을 지정합니다.
env -u PYTHONPATH uv run python tools/run_virtual_trial.py
```

출력된 URL을 열고 `key_file`의 키를 입력하면 1대·2대·스마트카트 계획을 선택할 수 있습니다. 가상 시험 초기화는 기존 기록을 보존하고 같은 URL에서 새 시험을 시작합니다. 새 가상 시험의 기본 속도는 0.08 m/s이며, Nav2는 장애물 없는 모사입니다. 자세한 범위·종료·오류 주입은 [가상 시험 가이드](docs/pinky-trial.md#10-실물-드라이버를-사용하는-가상-시험-환경)를 따릅니다.

### 서비스 작업 시험과 OMX 완료 입력

원본 TRIAL 화면의 **서비스 작업 시험**에서 물품 입고 또는 출고·인도 준비를 선택하고, 물품의 준비 상태·보관 장소를 확인한 뒤 **계획 확인 → 서비스 계획 적용 → 작업 실행**을 누릅니다. 서버가 기존 설정의 장소·방향·이동 모드로 계획을 생성하며 브라우저는 노드 경로를 제출하지 않습니다. 입고는 `two_pinky`의 물류 역할, 출고는 스마트카트 역할(1대 설정은 물류 역할)을 사용합니다. 고급 주행 시험은 기존 기능을 유지합니다.

가상 Pinky와 PR45 모의 OMX를 같은 격리 브로커에서 실행:

```bash
uv run python tools/run_virtual_trial.py --scenario two --omx mock
```

서비스 요청은 한 포장품·순차 시험이며 실행 중 새 요청은 거절합니다. 적용한 동일 요청은 재시도/재시작해도 기존 작업 ID를 반환하고, 같은 ID의 본문 변경을 거절합니다. 실행 전 기체 상태를 재검사합니다. 이 시험 요청은 서비스 서버 WSS 주문 계약이나 재고 DB를 구현한 것이 아닙니다.

OMX1·2 단계는 `roport-omx-task-v1`을 사용하며, UI 수동 완료와 MQTT 완료를 동일 정차 세션에 한 번만 반영합니다. 작업 전송 후 수동 확인은 접수 이벤트와 **접수 후 관측된 최신 Host 대기·팔 복귀 상태**가 필요합니다. 실행 중·결과 불명·오류는 수동으로 우회하지 않습니다. 완료 입력 출처와 모의/실물 실행 설정을 따로 기록합니다. PR45의 `arm_return_only`는 물품 위치·수량이나 고객 수령 증거가 아닙니다. OMX3 창고 상차는 수동 확인을 유지합니다.

수동 전용 기존 설정은 그대로 사용합니다. 외부 OMX Host를 연결할 때는 **새 trial.json과 journal**에 아래 `omx` 설정을 추가하고 Host와 site/control/binding을 맞춥니다. 설정 변경은 기존 이력에 덮어쓰지 않습니다. 별도 브로커는 필요하지 않습니다.

```json
{
  "omx": {
    "binding_id": "REPLACE_ME",
    "slot_id": "1a",
    "model_version": "REPLACE_ME",
    "calibration_version": "REPLACE_ME",
    "execution_mode": "mock"
  }
}
```

`execution_mode`는 실제 연결한 OMX 실행부를 나타냅니다. 현재 제공되는 `cooperative_host.py`는 mock 전용입니다. 실물 Pinky 설정과 OMX 실행 모드는 별개이며 실물 동작은 현장 설정·구동권·도킹 검증이 추가로 필요합니다. 생성한 resource_token은 격리 순차 시험의 상관 ID이며 운영 자원 예약/permit을 대신하지 않습니다. 관제 재시작·MQTT 단절·결과 timeout은 HOLD로 남기고 자동 재집기/재출발하지 않습니다. 시험 journal은 요청 32개 및 4 MiB 제한이 있으므로 보존·정리 후 새 이력으로 시험합니다.

## 서비스 WSS 관제 기반 (합의 전 후보)

Fleet은 `--journal` 또는 `ROPORT_DATABASE_URL` 중 하나를 선택한다. WSS는 DB와 `ROPORT_SERVICE_WSS_URL`, `ROPORT_SERVICE_TOKEN`, `ROPORT_SERVICE_SITE`를 모두 설정했을 때만 연결하며 DB migration은 별도로 수행한다. UI 접속·서비스 접수만으로 로봇 처리를 시작하지 않는다. 계약과 공용 JSON은 [WSS 후보](../../interfaces/wss/candidate/README.md)에 있다. S1 제안과는 호환되지 않으며 운영 계약은 양측 합의 대상이다.

- `adapters/wss/service.py`: Bearer WebSocket, 저장 후 ACK, 1초 재접속 대기, 최대 64 KiB 수신. 4001 종료는 자동 재접속하지 않는다.
- `adapters/db/service_tasks.py`: inbox·작업·outbox 원자 저장, 메시지별 거절 격리, 실행 스냅샷의 단일 DB owner.
- `contracts/service_wire.py`, `service_policy.py`: 수신 필드와 명세 갱신 규칙. DB 예외는 `ServiceStorageError`로 전달해 WSS가 DB 드라이버를 import하지 않는다.

형식 오류는 ACK하지 않고 다음 프레임을 처리한다. 유효한 envelope의 업무 거절은 `control_service_quarantine`에 site·message_id·내용 해시·오류 코드만 저장한 뒤 ACK한다. ACK는 **저장 확인**이며 업무 승인/수령 승인/출발 허가가 아니다. 같은 거절 재전송을 나중에 새 작업으로 실행하지 않는다. 서비스는 업무 성공을 `task.accepted`/진행으로 판단하고, 접수 없는 저장 ACK는 운영자가 격리 기록을 확인한다. 새 nack 규격은 추가하지 않았다.

서비스 저장소의 DB 접속·쿼리·commit 실패는 ACK하지 않고 1초 후 다시 연결한다. `OperationalStore`의 단일 owner 연결은 자동으로 재획득하지 않는다. 실행부가 그 연결 오류를 관찰하고 정지·복구 판단을 해야 한다. 운영 migration은 실행부를 멈춘 뒤 적용한다.

`dispatch_locked`는 배차 claim 또는 RUNNING 전환 시 고정한다. 배차 전 HOLD는 명세 수정 가능하며 설치 SQL 재실행도 이를 잠그지 않는다. outbox는 task 행 잠금 뒤 할당한 `seq`로 정렬해 한 번에 최대 100개를 읽고, ACK되지 않은 앞 묶음을 재전송한다. 서로 다른 작업의 전체 완료 순서는 보장하지 않으며 서비스는 높은 `task_revision`만 적용해야 한다.

수령의 최초 `arrival_verified=false`는 나중 도착/재전송으로 true가 되지 않는다. 조기 수령도 기록 ACK는 보내므로 서비스가 무조건 재전송하거나 성공으로 간주하면 안 된다. 관제 도착 증거와 서비스 수령이 불일치하면 서비스 담당/운영자가 대조하고, 도착 후 **새 수령 의사 확인**으로 새 receipt를 생성해야 한다. 이 후보에는 자동 대조/복구 UI가 없다.

테이블 설치는 명시적 CLI만 수행한다. 기존 기록을 보존하며 SQL 재실행을 지원하지만 운영 버전 migration runner를 대체하지 않는다.

```bash
# 폐기 가능한 시험 DB만 지정. 실제 접속 비밀값은 로컬 환경에서 주입.
ROPORT_DATABASE_URL='postgresql://localhost/roport_wss_test' \
  uv run python -m roport_control_server.adapters.db.service_tasks
ROPORT_TEST_DATABASE_URL='postgresql://localhost/roport_wss_test' \
  REQUIRE_REGISTRY_TESTS=1 PYTHONPATH=src uv run pytest -q tests/test_service_*.py
```

서비스 Java와의 로컬 상호운용은 [통합 시험](docs/service-web-local-test.md)으로 검사한다. 운영 TLS/프록시/인증·실물 로봇 검증은 후속 범위다.

서비스 작업은 `service_tick`이 주문의 포장품마다 안정적인 request_id로 Fleet에 제출한다. `customer_service`는 기본적으로 pickup_at 300초 전에 배차하며 `service_policy.dispatch_lead_s`로 시험 정책을 지정한다. 업무 package_id는 재고에 보존하고 선택 marker만 OMX 명령에 매핑한다. 빈 명세/미등록 재고/미보정 목표는 주행을 생성하지 않는다. 수령 증거가 확인된 포장품만 복귀한다. WSS worker의 예상 밖 종료는 owner에 전달하며 기존 STOP·도착 재확인·복구 조건은 유지한다.

현장 배포 묶음 생성, 체크섬 검증, MQTT 상태 수신 점검은 [Fleet 현장 배포 준비와 복구](../../docs/field/README.md)를 따른다.
