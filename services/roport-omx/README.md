# roport-omx

RO:PORT · GATE CREW — OMX 제어·협동 동작·시연 수집·학습

## 기술 스택

Python 3.12 코어 패키지 / uv / Ruff. 학습·정책 실행은 LeRobot·PyTorch를 사용하되 OMX 드라이버·CUDA 호환성을 확인한 별도 환경에서 버전을 고정합니다.

## 현재 범위

기존 협업 시험 Host와 제어판 실행 어댑터를 제공합니다. 기본 실행은 mock이며 실제 패널·모터·모델·보정은 현장에서 별도 확인해야 합니다.
CI 성공은 실로봇·GPU·통합 검증 완료를 의미하지 않습니다.
배포 환경 미정으로 CD와 자동 배포는 설정하지 않았습니다.

## 관련 문서

이 디렉토리는 단일 저장소의 구성요소입니다. 아래 명령은 이 디렉토리에서 실행합니다.

- [프로젝트 개요](https://github.com/kernel-robot-8th/kernel-robot-repo-1)
- [요구사항·설계 문서](../../docs/README.md)
- [Jira](https://fp-team1.atlassian.net/jira/projects/FFT1)

PR에 관련 Jira 키, 변경 내용, 검증 결과와 미검증 항목을 기록합니다.
실로봇 적용 전 장비·설정·캘리브레이션, 안전 정지·복구 절차를 확인합니다.

## 개발 방법

```bash
python3 -m pip install uv==0.13.5
uv sync --locked
uv run ruff check src
uv run python -c "import roport_omx"
uv build
```

## LeRobot 원본

[lerobot/](lerobot/)에 공식 `v0.6.1` 소스를 포함했습니다. 원본 커밋과 라이선스는 [UPSTREAM.md](UPSTREAM.md)에 기록합니다.

기존 코어 패키지와 별도 환경에서 설치합니다. 다음 명령은 소스 설치만 수행하며 로봇을 구동하지 않습니다.

```bash
cd lerobot
python -m pip install -e .
```

기본 CI 실행: `bash ci/test.sh` — 원본 구문·CLI 진입 함수·encoder 변환과 장비 검사 로직을 검증합니다.

장비 연결 검사는 전용 Python 환경에 `dynamixel-sdk==3.8.4 pyserial==3.5`를 설치한 뒤 실행합니다.

```bash
python scripts/check_hardware.py --port /dev/serial/by-id/YOUR_OMX_DEVICE --output /tmp/omx-check.json
```

기존 제어 프로그램이 시리얼 포트를 사용 중이면 함께 실행하지 마세요. PING·READ로 ID 11–16의 모델·현재 위치(encoder count)·토크·오류 상태만 읽습니다. 모터 설정·토크를 변경하지 않으며, 주행·관절 동작·캘리브레이션 정확도는 별도 현장 검증이 필요합니다. 검사 실패 시 exit code 1과 JSON 결과를 남깁니다.

## 최소화 규약 현장 Host (FFT1-180)

`field_host.py`는 `schema_version: 2.0` / `profile: roport-control-v2`를 사용합니다.
토픽은 `roport/{robot|workcell}/{target}/{cmd|event|ack}`이며 site/version 경로는 없습니다.
기존 `cooperative_host.py`의 v1 시험 규약은 별도 호환 경로로 유지합니다.

- `shelf_transfer_command`: OMX1+2 공동, N11, 가운데 열 `1b/2b`.
- `arm_action_command`: OMX1 단독 N21 `1a/2a`, OMX2 단독 N22 `1c/2c`. 각 슬롯·입출고 방향의 Runner/정책/캘리브레이션을 따로 주입합니다.
- `warehouse_load_command`: stock-omx(창고 OMX, MQTT 식별자 `omx3`) N01. 기존 `Omx3Host → Runner`를 실행하며 실제 결과의 안착·해제·복귀를 보고합니다. `package_size`는 임시 물품 카탈로그 값이며 추론으로 확인했다고 표시하지 않습니다.
- Host는 Pinky의 최신 정차·boot·도착 명령과 별도 정렬 센서, 해당 executor의 `model_version/calibration_version` 일치, 팔 준비를 검사합니다. 한 프로세스가 작업을 하나씩 소유합니다. 준비 검사·접수 저장이 지연되면 실행 직전에 만료를 재확인하며, Runner 내부 준비 중 만료도 첫 모터 목표 전 취소합니다. MQTT ACK와 팔 복귀만으로 물품 안착을 확정하지 않습니다.
- `(pinky_id, arrival_command_id)`는 이송 접수와 함께 journal에 소모 기록을 저장합니다. 같은 명령 재전송은 저장 결과만 반환하며, 같은 도착을 새 명령으로 재사용하면 `arrival_already_used`입니다. 실패·ACK·Host 재시작 뒤에도 소모 기록을 유지합니다. 사전 거절은 소모하지 않지만 접수 이후 실패/만료는 소모합니다. 재시도는 현장 상태 조정 후 **새 이동으로 확인된 도착**이 필요하며 ID만 바꾸거나 journal을 지워서 재시도하지 않습니다.
- 공동 명령은 `shelf-01`과 참여 팔 `omx1/omx2`에 같은 `active_command_id`를 표시하며 팔 관측도 공동 executor에서 읽습니다. 무관한 대상은 `active_command_id: null`, `ready: false`, `fault: host_busy`입니다.
- 완료는 `command_status`로 보고하고 `event_stored`까지 journal/outbox에 유지합니다. 재시작·한쪽 팔 미복귀·관측 불명·watchdog 만료는 HOLD입니다. 정지 대상은 실패·재시작 후에도 보존하고 재시작 시 abort를 요청합니다. 새 boot를 조회한 뒤 기존 `target_command_id`로 정지를 재요청할 수 있습니다. 정지 중 다른 정지 명령은 `stop_already_pending`으로 거절하며 원래 요청을 덮어쓰지 않습니다. 정지 성공은 `stop_status`의 최신 관측으로만 확정하고 HOLD는 유지합니다.

가상 모드(격리 loopback 브로커 필요):

```bash
uv run python scripts/field_host.py --mock --host 127.0.0.1 --port 1883 \
  --journal ~/.local/state/roport/field-omx.json
```

가상 모델/캘리브레이션 값은 `sim-model` / `sim-cal`입니다. `--package-sizes PATH`로
`{"package-small-1":"small","package-large-1":"large"}` 시험 카탈로그를 넣을 수 있습니다.
하드웨어는 `--factory module:function --hardware-authorized`로 명시하며 factory가
`(executors, alignment_sensor)`를 반환해야 합니다. 공용 모터 버스를 여러 드라이버가
중복 소유하지 않도록 현장 장치 소유자를 재사용합니다.

`WarehouseExecutor`와 `IndependentArmExecutor`에는 실제 Robot/Policy/Vision 포트와
확정된 모델·캘리브레이션 버전이 필요합니다. 공동 `PanelFieldExecutor`는 실제 중지
콜백과, command/package/slot/arrival에 일치하는 최신 안착·해제 센서 콜백이 추가로
필요합니다. 패널의 `arm_return_only`/`shelf_align`만으로 재고 확정은 불가능합니다.
개별 `return_home`은 스키마에 예약된 동작이며 현재 Host는 항상 `unsupported_operation`으로 거절합니다.
홈 밖 복귀 경로는 아직 구현하지 않았고 모델/ArUco·정렬/안착 센서 바인딩도 현장 검증이 필요합니다.

검사: `bash ci/test.sh` 및 관제 서버의 `tests/test_omx_device_mqtt.py`,
`tests/test_omx_panel_integration.py`. 루프백 MQTT·실제 Host/Runner/PanelExecutor를
재사용하고 모터·정책·카메라/센서 입출력만 모사합니다. 현장 센서·모터·네트워크 성능은
별도 검증 대상입니다. AI UDP의 inference/request 상관관계 ID는 MQTT 최소화와 별개로 유지합니다.

### 정지 관측과 실물 적용 전 남은 조건

- 모든 실물 어댑터의 `stop_status()`는 모터 관측에서 `{"observed_at": "ISO-8601 UTC", "stopped": true/false}`를 반환합니다. 공동 작업은 양팔 모두 정지한 값이어야 합니다. 어댑터 PC와 같은 시각 기준의 관측만 받으며 허용 나이는 0~3초입니다. 정지 완료는 **해당 abort 요청 이후** 관측이어야 합니다. 콜백은 최신 센서 캐시를 짧게 읽고, 홈 위치·HTTP 성공·명령 접수를 정지 근거로 만들지 않습니다. 누락·오류·오래된 값은 준비/정지 확인으로 인정하지 않습니다.
- `WarehouseExecutor`는 진행 중 Runner에 취소를 전달하고, 재시작 등 Runner가 없으면 기존 Robot 포트의 `hold()`를 호출합니다. `IndependentArmExecutor`도 같은 경로를 사용합니다. `PanelFieldExecutor`는 현장 `stop` 콜백을 호출합니다. 관측이 없으면 `physical_stop_not_confirmed` + HOLD이며, 정지 관측 연결 전에는 실물 준비 상태가 되지 않습니다.
- v2 `PanelFieldExecutor`는 준비 조회·시작·결과 조회를 MQTT owner 밖에서 직렬 실행하고, 정지는 별도 작업자로 전달합니다. 모델 목록과 팔 준비를 먼저 조회하며, owner는 캐시만 읽고, 실제 POST 직전에는 worker가 팔 준비·정지·만료를 다시 확인합니다. 읽기 시작 시각부터 3초가 지난 관측은 준비/완료 근거로 재발행하지 않습니다. `stop_status`와 `completion`은 현장의 최신 센서 캐시를 짧게 읽는 콜백이어야 합니다. 작업 중 패널이 생략하는 팔 관측은 미확인으로 표시하며, 실제 조회 실패/장치 오류와 구분합니다. 생략된 관측으로 준비·정지·복귀를 확정하지 않습니다. v1 `--executor panel`의 직접 HTTP 경로는 그대로입니다.
- 정지/종료가 대기 중인 시작보다 먼저 처리되면 POST를 보내지 않습니다. 이미 POST 중이면 결과가 돌아올 때 정지를 다시 요청하고 그 이후 정지 관측을 요구합니다. 시작 응답이 유실되면 `panel_start_unconfirmed`로 HOLD하며, 정지 센서가 true여도 미확인 시작을 정지 완료로 덮어쓰지 않습니다. 자동 재시작·재전송은 없습니다.
- **패널 기반 v2 실물 factory는 아직 운영에 연결하지 않습니다.** 기존 패널 API는 클라이언트 timeout 뒤 늦은 시작을 취소한다고 보장하지 않습니다. 로컬 지연 시험에서도 이 경우를 재현했으며, 스레드 분리는 원격 취소 보장이 아닙니다. 아래 임시 HTTP 계약과 송수신 모듈을 준비했습니다. 외부 패널 담당자가 실제 프로세스 시작 경계에 수신부를 붙여야 하며, 현재 공개 패널에 이 API가 구현됐다는 의미가 아닙니다. MQTT 필드와 실물 factory의 장치 경계는 유지했습니다. 센서·모터·실물 캘리브레이션도 별도 검증 대상입니다.
- OMX의 새 물리 명령은 Pinky와 동일하게 `sent_at` 나이 −0.5~3초를 검사하고 벗어나면 `stale_message`로 거절합니다. 기록된 명령의 동일 본문 재전송은 신선도 검사 전에 저장 결과를 반환합니다. 만료는 수신 PC의 `expires_at` 절대시각 기준입니다. 유지 확인은 `sent_at`의 나이 −1~1초, Pinky/정렬 관측은 −1~3초, 로컬 정지 관측은 0~3초를 검사합니다. 명령과 센서의 허용 창이 다르므로 관제·로봇 NTP와 센서 timestamp 기준을 맞춰야 합니다.
- HOLD 자동 해제는 제공하지 않습니다. 관제가 요청한 정지 완료를 먼저 확인하고 실제 물품 제거·팔 복귀·유휴 상태를 정리한 다음, 같은 journal로 Host를 재시작하면서 `--reconcile-note "현장 확인 내용"`를 붙일 수 있습니다. 미완료 명령/정지가 남거나 장치가 정지·복귀·빈 그리퍼를 확인하지 못하면 실패합니다. 명령 이력/outbox를 지우지 않고 복구 기록만 남기며, MQTT 작업 재시작은 하지 않습니다. PR71 OMX3 Host의 RECONCILING도 기존 `clear_reconciling(key)` 절차로 먼저 정리해야 합니다. 이어 관제 UI에서 저장 결과를 대조하고 보류 해제와 실행을 각각 승인합니다. journal 삭제를 복구 방법으로 사용하지 않습니다.
- 같은 `command_id`로 다른 본문을 보내거나 스키마·출처·토픽이 틀린 패킷은 로그 후 거절합니다. 기존 명령의 결과를 덮어쓸 새 상태 이벤트는 만들지 않습니다. 관제는 유효한 상태/기존 결과를 조회해 확인합니다.

`stock-omx`는 창고 팔의 역할 명칭입니다. MQTT 토픽·JSON·journal 및 Python `omx3` 모듈 이름은 기존 연동을 위해 유지합니다. 식별자까지 바꾸는 작업은 관제·로봇·명세를 함께 전환하는 별도 변경입니다.

### 임시 command 단위 패널 HTTP 계약

실제 장비 실행 프로세스를 소유한 패널 한 곳에서 `PanelCommands`를 사용합니다. 기존 원격 `/api/eval/start`를 다른 서버에서 감싸는 프록시는 늦은 실행을 취소할 수 없으므로 지원 방식이 아닙니다. 수신 경계는 `src/roport_omx/panel_commands.py`, 선택형 클라이언트는 `managed_panel.py`이며 기존 `PanelExecutor`는 유지합니다. 외부 패널 팀의 API가 확정되면 이 경계만 교체하고 MQTT 명령 ID/장치 센서 계약은 유지합니다.

| HTTP | 입출력 / 연결할 메서드 |
| --- | --- |
| GET `/api/control/capabilities` | `{ "command_cancel": "durable" }`; 실제 아래 동작을 구현한 패널만 응답 |
| POST `/api/control/commands` | `{command_id, expires_at, payload}` → `ledger.submit(body)`; payload는 기존 eval 옵션, expires_at은 timezone 포함 ISO 시각 |
| GET `/api/control/commands/{command_id}` | `ledger.get(id)` → `{command_id, state, observation}`; 없음은 `not_found` |
| POST `/api/control/commands/{command_id}/cancel` | `{}` → `ledger.cancel(id, actual_stop)`; 아직 접수 전이어도 취소 tombstone을 저장 |

1. 패널 시작 시 **동일 SQLite 파일과 단일 장치 소유 프로세스**로 ledger를 엽니다. 받은 payload의 모델/장비 옵션은 기존 패널 검증을 그대로 거칩니다. 수신 크기 제한·인증은 패널 HTTP 경계 책임입니다.
2. 새 요청을 `submit`으로 저장한 뒤 실제 장비 프로세스 생성 직전에 `start(id, begin)`을 호출합니다. `begin(payload)`는 로컬 프로세스를 생성하고 반환해야 합니다. 원격 큐에 위임하지 않습니다. 만료를 다시 확인하고, 취소된 ID나 동일 ID 재전송으로 두 번 실행하지 않습니다. 다른 내용의 ID 재사용은 충돌로 응답합니다.
3. 실행 결과는 기존 센서 결과를 `finish(id, observation)`으로 저장합니다. 완료는 `state=finished`, 실패는 `state=failed`이고 arms 등 기존 결과 구조를 보존합니다. `completed/failed` 조회는 이 저장 관측을 반환합니다.
4. 취소 요청은 running을 stopping으로 저장하고 실제 stop을 호출합니다. 이후 `confirm_stopped(id, {observed_at, stopped})`에 **취소 이후 0~3초 이내 실제 모터 정지 관측**이 들어와야 cancelled가 됩니다. pending/미접수 ID는 즉시 cancelled로 보존합니다. 늦은 start/finish는 취소를 되돌리지 않습니다.
5. 재시작 시 미종료 명령은 unknown이며 자동 재실행하지 않습니다. 같은 ID 취소와 정지 관측을 완료해야 다음 작업을 받습니다. `ManagedPanelExecutor`를 `PanelFieldExecutor`에 주입하면 명령별 취소/조회를 사용하고, HTTP 취소가 지연되어도 현장 정지 콜백을 먼저 호출합니다. 원격 취소 확정과 로컬 정지 관측을 모두 만족해야 정지 완료입니다.

모의 HTTP/SQLite 시험은 `ci/test_panel_commands.py`, 지연·타임아웃·취소와 로컬 정지는 `ci/test_panel_async.py`입니다. 외부 패널 배포·공동 양팔 실구동·모터 정지 센서 연결은 이 테스트에 포함하지 않습니다. `shelf-01`은 공동작업 controller이며, 개별 팔 상태 확인은 OMX1/2 각각에 적용합니다.

### PR #66 기능 인수 범위

`cooperative_host.py --executor panel`, 슬롯별 Route/출력 필터/eval 옵션, 두 예제 설정,
모든 경로 모델의 실제 팔 관측, `end_on_return`, 작업 결과·일반 중지/비상 정지 구분을 유지합니다.
관련 패널 회귀 테스트도 함께 유지하며 만료·미복귀 판정만 위 기준으로 수정했습니다.
별도 저장소의 제어판(3.12.0 이상) 설치·자동 Host 시작 설정과 현장 정책/캘리브레이션은
이 저장소의 PR 병합만으로 설치되지 않습니다. 구형 v1 시험과 v2 현장 규약을 혼용하지 않습니다.

## 현장 적용 준비

[OMX 현장 매뉴얼](../../docs/field/omx.md)에 배포·factory 연결·센서 계약·실행 순서를 정리했다. `panel_command_api.PanelCommandAPI`는 기존 패널 HTTP 라우터에 붙일 수신부이며 실제 프로세스 소유자 내부에 설치해야 한다. 상태 보고만 받는 관제 `tools/check_field.py`는 명령이나 작업 ACK를 발행하지 않는다. Host client ID는 담당 target 목록으로 구분하므로 창고/인도장 Host를 분리해도 서로 연결을 끊지 않는다.

## shelf-01 협업 Host 연결 (시험 단계, FFT1-153)

OMX1·2 COOPERATIVE 작업 명령(`roport-omx-task-v1`, `roport/v1/{site}/workcell/shelf-01/{cmd,event,ack}`)을 받아 실행부로 넘기고 event를 회신합니다. MQTT 송수신은 관제 서버 `adapters/mqtt`를 `src/roport_omx/mqtt_link/`로 복사해 사용합니다(원본 커밋은 각 파일 첫 줄). 기본값은 모의 실행부이며, `--executor panel`을 지정한 경우에만 로컬 패널을 호출합니다.

```bash
# Host (journal은 재시작 후에도 남는 경로, /tmp 금지)
uv run python scripts/cooperative_host.py --journal ~/.local/state/roport/test-shelf-01.json
# 시험용 관제 송신기: status → shelf_inbound → event.ack
uv run python scripts/cooperative_control_sim.py --host BROKER_HOST inbound
```

- 브로커 주소·TLS·계정은 관제와 같은 `ROPORT_MQTT_*` 환경변수로 지정합니다. 실제 값은 커밋하지 않습니다.
- 시험 합의: 실행부가 armed일 때만 시작합니다. 완료는 양팔 홈 복귀·그리퍼 열림으로 판정하며 물품 안착은 관측하지 않아 `placement_confirmed`·`quantity_confirmed`·`package_location`을 `null`로 보고합니다.
- `slot_id`는 shelf-01 2행 3열 칸 `1a 1b 1c 2a 2b 2c`만 허용합니다(1 = 아래층, 2 = 위층, a·b·c = 팔 쪽에서 왼쪽부터, ArUco S1A~S2C = ID 28~33). 그 밖의 값은 `UNKNOWN_SLOT`으로 거절합니다.
- `cmd` 토픽은 `*.requested`, `ack` 토픽은 `event.ack`만 받고 QoS 1·비retained가 아니면 응답 없이 버립니다.
- journal에는 site·control·binding을 기록하며 다른 등록 정보로 열면 시작을 거부합니다. 등록 정보를 바꾸면 별도 `--journal`을 사용합니다. journal 저장에 실패하면 수신 확인(ACK) 없이 종료하고, 재시작 시 journal 기준으로 복구합니다.
- 실패·재시작으로 결과가 불명확하면 HOLD가 되어 새 작업을 거절합니다. 팔·물품·Pinky를 확인한 뒤 `--clear-hold`로 해제합니다.
- 실물 실행: `--executor panel --routes <경로 파일> --box-marker <상자 ID>`는 같은 PC의 OMX 제어판(v3.12.0 이상) API로 평가 1회를 시작합니다. 칸마다 모델이 다르므로 `(operation, slot_id)`별 모델·작업 문장·시간을 경로 파일에 둡니다(`config/cooperative_routes.example.json`). 경로마다 제어판 출력 필터(`output_filter`: 약·중·강, 생략하면 끔)와 `filter_gripper`를 지정하면 관제로 시작한 평가에도 그대로 적용합니다. 경로가 없는 칸, 학습 목록에 없는 모델, 제어판 armed 꺼짐은 동작 전에 거절합니다. 완료는 제어판 `last_result`의 정상 종료·양팔 복귀·그리퍼 놓음(52+5% = 54.6 이하)으로 판정합니다. 제어판 3.12부터 Host는 `end_on_return`을 보내, 팔이 처음 자세에서 벗어났다가 돌아오고(모든 관절 8 이내) 그리퍼가 54.6 이하로 0.3초 유지되면 `episode_s` 전이라도 평가를 끝냅니다(정책의 재시도 방지). 조기 종료 신호(`last_result.last_episode.returned`)만으로 안전 복귀를 확정하지 않습니다. 양팔 `at_home is True`와 그리퍼 놓음이 확인될 때만 `evidence: "arm_return_only"`로 완료하며 False/미관측은 실패·HOLD입니다. 제어판 3.9부터 평가 뒤 선반 정렬이 이어지면 `final`이 될 때까지 기다리고, 정렬 성공·정렬 뒤 쉬는 자세 확인(`align.rest`)·최종 `/api/arms` 홈/그리퍼 실측을 함께 판정합니다(`evidence: "shelf_align"`). 사람 조치를 기다리는 `hold`(비상 정지, 내려놓기 경로 없음)는 기다리지 않고 바로 실패·HOLD로 보고합니다. 제어판에서 사람이 누른 중지·즉시 중단은 정지가 끝난 뒤 `PANEL_STOPPED`·`PANEL_ABORTED`(정렬 중이면 `PANEL_ALIGN_STOPPED`)로 실패 보고하되 HOLD하지 않습니다. 정지하면 제어판 armed가 꺼지므로 다음 시작은 사람이 armed를 다시 켜야 합니다. 비상 정지·강제 종료·내려놓기 실패·오류·미복귀는 HOLD합니다. 상태 조회(`status.requested`)의 양팔 상태는 제어판 3.11의 `GET /api/arms?model=…`(실행 중이 아닐 때 모터 위치를 읽음)에서 가져오며, 경로표의 모든 모델 홈 자세 안이어야 `returned`입니다. 그리퍼는 54.6 이하면 놓음, 넘으면 `grasping`으로 봅니다(2026-11-10 시험 합의: 빈 채로 닫히면 48~51, 상자에 막히면 58~60, 열림 61~67. 상자 모서리를 50 근처에서 쥘 수도 있어 놓음 여부는 팔 복귀와 함께 판정). 제어판 3.11부터 armed를 켤 때 `~/.config/roport/omx-host.json` 설정(예: `config/omx-host.example.json`, `link_config`는 `execution_mode: hardware`여야 함)으로 이 Host를 자동 시작하고, armed를 꺼도 Host는 남아 보고·상태 조회에 답합니다. Host는 제어판 **3.12.0 이상**만 준비 상태로 보고합니다(낮으면 `executor_error: PANEL_VERSION`). **팔이 움직이므로 사람이 옆에서 armed를 켠 상태로만 시험합니다.** 상자 ID는 시험용 고정값이며 `package_id` 규칙은 관제와 합의 후 연결합니다.
- 준비 조회·모델 검사 이후, 접수 journal 저장 이후와 `/api/eval/start` POST 직전에 각각 `expires_at`을 재검사합니다. 만료되면 새 평가를 시작하지 않습니다. HTTP 시작 응답이 끊기면 결과 불명·HOLD이므로 자동 재시작하지 않습니다.
- 검사: `bash ci/test.sh` (`ci/test_cooperative.py`, `ci/test_panel_executor.py`; 브로커·제어판·장비 불필요). 실제 브로커 왕복은 로컬 Mosquitto에서만 확인했고, 패널 연동·실물 팔은 미검증입니다.
