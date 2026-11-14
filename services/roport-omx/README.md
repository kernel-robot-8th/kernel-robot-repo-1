# roport-omx

RO:PORT · GATE CREW — OMX 제어·협동 동작·시연 수집·학습

## 기술 스택

Python 3.12 코어 패키지 / uv / Ruff. 학습·정책 실행은 LeRobot·PyTorch를 사용하되 OMX 드라이버·CUDA 호환성을 확인한 별도 환경에서 버전을 고정합니다.

## 현재 범위

기존 협업 시험 Host와 패널 실행 어댑터, OMX3 Runner, 최소화 규약 현장 Host를 제공합니다. 아래 현장 Host는 가상 장치와 실제 MQTT로 검증하며 실장비 연결은 별도 factory와 현장 설정이 필요합니다.
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
- 실물 실행: `--executor panel --routes <경로 파일> --box-marker <상자 ID>`는 같은 PC의 OMX 제어판(v3.12.0 이상) API로 평가 1회를 시작합니다. 칸마다 모델이 다르므로 `(operation, slot_id)`별 모델·작업 문장·시간을 경로 파일에 둡니다(`config/cooperative_routes.example.json`). 경로마다 제어판 출력 필터(`output_filter`: 약·중·강, 생략하면 끔)와 `filter_gripper`를 지정하면 관제로 시작한 평가에도 그대로 적용합니다. 경로가 없는 칸, 학습 목록에 없는 모델, 제어판 armed 꺼짐은 동작 전에 거절합니다. 완료는 제어판 `last_result`의 정상 종료·양팔 복귀·그리퍼 놓음(52+5% = 54.6 이하)으로 판정합니다. 제어판 3.12부터 Host는 `end_on_return`을 보내, 팔이 처음 자세에서 벗어났다가 돌아오고(모든 관절 8 이내) 그리퍼가 54.6 이하로 0.3초 유지되면 `episode_s` 전이라도 평가를 끝냅니다(정책의 재시도 방지). 이 경우 복귀는 그 감지로 확인하며(제어판 `last_result.last_episode.returned`), 완료 근거는 관제가 받는 `evidence: "arm_return_only"` 그대로 보고합니다. 제어판 3.9부터 평가 뒤 선반 정렬이 이어지면 `final`이 될 때까지 기다리고, 정렬 성공·정렬 뒤 쉬는 자세 확인(`align.rest`)·평가 종료 시 그리퍼 열림으로 판정합니다(`evidence: "shelf_align"`). 사람 조치를 기다리는 `hold`(비상 정지, 내려놓기 경로 없음)는 기다리지 않고 바로 실패·HOLD로 보고합니다. 제어판에서 사람이 누른 중지·즉시 중단은 정지가 끝난 뒤 `PANEL_STOPPED`·`PANEL_ABORTED`(정렬 중이면 `PANEL_ALIGN_STOPPED`)로 실패 보고하되 HOLD하지 않습니다. 정지하면 제어판 armed가 꺼지므로 다음 시작은 사람이 armed를 다시 켜야 합니다. 비상 정지·강제 종료·내려놓기 실패·오류·미복귀는 HOLD합니다. 상태 조회(`status.requested`)의 양팔 상태는 제어판 3.11의 `GET /api/arms?model=…`(실행 중이 아닐 때 모터 위치를 읽음)에서 가져오며, 경로표의 모든 모델 홈 자세 안이어야 `returned`입니다. 그리퍼는 54.6 이하면 놓음, 넘으면 `grasping`으로 봅니다(2026-11-10 시험 합의: 빈 채로 닫히면 48~51, 상자에 막히면 58~60, 열림 61~67. 상자 모서리를 50 근처에서 쥘 수도 있어 놓음 여부는 팔 복귀와 함께 판정). 제어판 3.11부터 armed를 켤 때 `~/.config/roport/omx-host.json` 설정(예: `config/omx-host.example.json`, `link_config`는 `execution_mode: hardware`여야 함)으로 이 Host를 자동 시작하고, armed를 꺼도 Host는 남아 보고·상태 조회에 답합니다. Host는 제어판 **3.12.0 이상**만 준비 상태로 보고합니다(낮으면 `executor_error: PANEL_VERSION`). **팔이 움직이므로 사람이 옆에서 armed를 켠 상태로만 시험합니다.** 상자 ID는 시험용 고정값이며 `package_id` 규칙은 관제와 합의 후 연결합니다.
- 검사: `bash ci/test.sh` (`ci/test_cooperative.py`, `ci/test_panel_executor.py`; 브로커·제어판·장비 불필요). 실제 브로커 왕복은 로컬 Mosquitto에서만 확인했고, 패널 연동·실물 팔은 미검증입니다.

## 최소화 규약 현장 Host (FFT1-180)

`field_host.py`는 `schema_version: 2.0` / `profile: roport-control-v2`를 사용합니다.
토픽은 `roport/{robot|workcell}/{target}/{cmd|event|ack}`이며 site/version 경로는 없습니다.
기존 `cooperative_host.py`의 v1 시험 규약은 별도 호환 경로로 유지합니다.

- `shelf_transfer_command`: OMX1+2 공동, N11, 가운데 열 `1b/2b`.
- `arm_action_command`: OMX1 단독 N21 `1a/2a`, OMX2 단독 N22 `1c/2c`. 각 슬롯·입출고 방향의 Runner/정책/캘리브레이션을 따로 주입합니다.
- `warehouse_load_command`: stock-omx(창고 OMX, MQTT 식별자 `omx3`) N01. 기존 `Omx3Host → Runner`를 실행하며 실제 결과의 안착·해제·복귀를 보고합니다. `package_size`는 임시 물품 카탈로그 값이며 추론으로 확인했다고 표시하지 않습니다.
- Host는 Pinky의 최신 정차·boot·도착 명령과 별도 정렬 센서, 해당 executor의 `model_version/calibration_version` 일치, 팔 준비를 검사합니다. 한 프로세스가 작업을 하나씩 소유합니다. 준비 검사·접수 저장이 지연되면 실행 직전에 만료를 재확인하며, Runner 내부 준비 중 만료도 첫 모터 목표 전 취소합니다. MQTT ACK와 팔 복귀만으로 물품 안착을 확정하지 않습니다.
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
개별 `return_home`의 실장비 실행 경로, 모델/ArUco·정렬/안착 센서 바인딩은 아직 현장
설정이 없으므로 미검증이며 factory에서 준비되지 않은 동작은 거절합니다.

검사: `bash ci/test.sh` 및 관제 서버의 `tests/test_omx_device_mqtt.py`,
`tests/test_omx_panel_integration.py`. 루프백 MQTT·실제 Host/Runner/PanelExecutor를
재사용하고 모터·정책·카메라/센서 입출력만 모사합니다. 현장 센서·모터·네트워크 성능은
별도 검증 대상입니다. AI UDP의 inference/request 상관관계 ID는 MQTT 최소화와 별개로 유지합니다.

### 정지 관측과 실물 적용 전 남은 조건

- 모든 실물 어댑터의 `stop_status()`는 모터 관측에서 `{"observed_at": "ISO-8601 UTC", "stopped": true/false}`를 반환합니다. 공동 작업은 양팔 모두 정지한 값이어야 합니다. 어댑터 PC와 같은 시각 기준의 관측만 받으며 허용 나이는 0~3초입니다. 정지 완료는 **해당 abort 요청 이후** 관측이어야 합니다. 콜백은 최신 센서 캐시를 짧게 읽고, 홈 위치·HTTP 성공·명령 접수를 정지 근거로 만들지 않습니다. 누락·오류·오래된 값은 준비/정지 확인으로 인정하지 않습니다.
- `WarehouseExecutor`는 진행 중 Runner에 취소를 전달하고, 재시작 등 Runner가 없으면 기존 Robot 포트의 `hold()`를 호출합니다. `IndependentArmExecutor`도 같은 경로를 사용합니다. `PanelFieldExecutor`는 현장 `stop` 콜백을 호출합니다. 관측이 없으면 `physical_stop_not_confirmed` + HOLD이며, 정지 관측 연결 전에는 실물 준비 상태가 되지 않습니다.
- **실물 실행은 아직 검증 완료가 아닙니다.** 패널 HTTP 호출은 여전히 동기식이며 `/api/arms` 최대 15초·상태 조회 5초·시작 300초가 MQTT 처리와 3초 watchdog을 지연시킬 수 있습니다. 현장 적용 전에 패널의 취소 가능한 비동기 시작·센서 캐시·독립 정지 경로를 연결하고 지연/정지 시험을 해야 합니다. 스레드로 감싸는 것만으로 원격 시작 취소가 보장되지는 않습니다.
- OMX의 물리 명령 만료는 수신 PC의 `expires_at` 절대시각 기준입니다. 유지 확인은 `sent_at`의 나이 −1~1초, Pinky/정렬 관측은 −1~3초, 로컬 정지 관측은 0~3초를 검사합니다. Pinky의 명령 신선도 창과 다르므로 관제·로봇 NTP와 센서 timestamp 기준을 맞춰야 합니다.
- HOLD 자동 해제/원격 해제는 제공하지 않습니다. 물품·팔·진행 중 패널 작업의 현장 조정과 명시적 복구 절차는 후속입니다. journal 삭제를 복구 방법으로 사용하지 않습니다.
- 같은 `command_id`로 다른 본문을 보내거나 스키마·출처·토픽이 틀린 패킷은 로그 후 거절합니다. 기존 명령의 결과를 덮어쓸 새 상태 이벤트는 만들지 않습니다. 관제는 유효한 상태/기존 결과를 조회해 확인합니다.

`stock-omx`는 창고 팔의 역할 명칭입니다. MQTT 토픽·JSON·journal 및 Python `omx3` 모듈 이름은 기존 연동을 위해 유지합니다. 식별자까지 바꾸는 작업은 관제·로봇·명세를 함께 전환하는 별도 변경입니다.
