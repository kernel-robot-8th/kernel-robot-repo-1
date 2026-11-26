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
- [OMX 현장 운영 안내](docs/omx-operations.md): 제어판 v3·On-site 패널 받기·설치·사용
- [제어판 연동 기준](docs/control-panel.md): Host가 쓰는 제어판 API와 검증 버전

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
- `warehouse_load_command`: stock-omx(창고 OMX, MQTT 식별자 `omx3`) N01. 기존 `Omx3Host → Runner`를 실행하며 실제 결과의 안착·해제·복귀를 보고합니다. 상차 대상은 모델이 학습한 마커 `15,16,17,21,22`만 받습니다. `19`·`20`은 물품 마커이지만 이 모델의 상차 대상은 아닙니다. `package_size`는 임시 물품 카탈로그 값이며 추론으로 확인했다고 표시하지 않습니다.
- Host는 Pinky의 최신 정차·boot·도착 명령, 해당 executor의 `model_version/calibration_version` 일치, 팔 준비를 검사합니다. 한 프로세스가 작업을 하나씩 소유합니다. 준비 검사·접수 저장이 지연되면 실행 직전에 만료를 재확인하며, Runner 내부 준비 중 만료도 첫 모터 목표 전 취소합니다. MQTT ACK와 팔 복귀만으로 물품 안착을 확정하지 않습니다. OMX1·2의 개별/공동 이송은 별도 정렬 센서도 검사합니다.
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

### Factory 정렬 센서

**OMX3 N01 상차는 별도 정렬 센서를 사용하지 않습니다.** 쟁반에는 ID 18을 포함해 마커를
붙이지 않습니다([마커 기준](../../docs/aruco/README.md)). `warehouse_load_command` 접수·실행 중에는
Pinky의 최신 N01 정차·도착·boot·라인 비활성·주행 소유권 해제를 계속 확인합니다. Runner가
완료 직전 **상자 마커의 쟁반 영역 안착·그리퍼 해제·팔 복귀**를 확인하며, `VERIFY_FAILED`는
`command_status(status="failed", reason="VERIFY_FAILED")`로 관제에 전달됩니다. 관제는 작업을
HOLD하고 Pinky의 다음 주행을 발행하지 않습니다. OMX3만 실행하는 factory는 `(executors, None)`을
반환할 수 있습니다. 이 기준은 학습된 N01 도킹 위치·보정값이 현장에 적용된 것을 전제로 합니다.

OMX3 factory는 `--factory roport_omx.omx3.field:build`입니다. `ROPORT_OMX3_INFER_SERVER`
(`host:port`), `ROPORT_OMX3_MODEL`(GPU PC 체크포인트 경로), `ROPORT_OMX3_PACKAGE_SIZES`(상자 크기
JSON, 형식: `{"15": "small", "16": "large", ...}`. 값은 관제와 합의한 상자별 크기)가 필요합니다.
크기 파일은 학습한 마커 15·16·17·21·22를 빠짐없이 `small`·`large`로 담아야 하며, 아니면 GPU·팔을
쓰기 전에 시작을 거절합니다(관제 선반 배정이 이 값을 씁니다). 포트·카메라·키 파일은
`ROPORT_OMX3_PORT`·`ROPORT_OMX3_CAMERA`·`ROPORT_OMX3_KEY_FILE`로 바꿀 수 있습니다. 모델을
GPU 서버에 먼저 올린 뒤 팔을 연결하며, 연결할 때 토크를 씁니다. 정책은 OMX 제어판 "부드럽게 실행"과
같은 시간 앙상블(ACT, 계수 0.01)로 돕니다. 앞 추론 응답이 오는 대로 새로 예측하고(UDP 왕복 약 40 ms라
1~2스텝마다, 초당 약 20~25건), 같은 스텝을 가리키는 예측을 평균해 동작 묶음 교체 때의 튐을 없앱니다. `stopped`는 Runner와 상태 조회가
이미 읽은 관절 값으로 판정합니다. 자세 유지(`hold`) 이후 0.5초 넘게 모든 관절 변화가 0.3 이하이면
정지입니다. 정지 요청(`abort`)부터 hold가 성공할 때까지의 관측은 버립니다(hold가 실패하면 계속 버려
정지로 보지 않습니다). 실측한 정지 떨림은 최대
0.049였습니다. VERIFY 직후에는 정지 표본이 없으므로, Runner가 끝난 뒤 executor가 직접 관측을 모으는
동안 `running`(`VERIFY`)으로 보고합니다. 정지가 확인되면 `finished`로 바뀌고, 3초 안에 확인되지 않으면
`OMX3_STOP_UNCONFIRMED` 실패입니다. 종료할 때 `close()`는 Runner를 취소하고 끝날 때까지 기다립니다.
Runner가 끝나지 않으면 5초마다 로그를 남기며 계속 기다리고, 팔 소유권도 놓지 않습니다. 끝난 뒤
정지가 확인되어야 연결을 닫습니다. 3초 안에 확인되지 않으면 hold를 다시 쓰고 계속 기다리며, 그래도
멈추지 않으면 현장에서 비상 정지·강제 종료로 처리합니다. 토크는 끄지 않습니다. `model_version`은
체크포인트 폴더 이름이고 `calibration_version`은 `robot_id-캘리브레이션 파일 sha256 앞 8자리`입니다.
lerobot·OpenCV·paho-mqtt가 함께 있는 장비 환경이 필요하며, 실물 팔과 실브로커로는 아직 검증하지
않았습니다.

아래 정렬 센서 계약은 **OMX1·2 개별 이송과 `shelf-01` 공동 이송**에 적용합니다.

센서 호출은 `alignment_sensor(command, pinky_observation)`입니다. 기존 한 인자 센서는
두 인자로 수정해야 합니다. factory 자체는 계속 인자 없이 호출되고 `(executors, sensor)`를
반환합니다. mock과 factory 모두 같은 Host 호출 경로를 사용하며 별도 MQTT 구독은 필요 없습니다.

Host가 정차·도착·boot·최신성을 검증한 뒤, 해당 Pinky의 `robot_state.payload`에 `pinky_id`를
붙여 깊은 복사본으로 전달합니다. 센서가 받은 dict를 바꿔도 Host의 원본 관측은 바뀌지 않습니다.

| 센서가 읽는 값 | 반환 기록에 사용할 값 |
| --- | --- |
| `pinky_observation["pinky_id"]` | `pinky_id` |
| `pinky_observation["boot_version"]` | `pinky_boot_version` |
| `pinky_observation["state"]["arrival_command_id"]` | `arrival_command_id` |
| `pinky_observation["state"]` | 검증된 `node`, `stopped`, `line_enabled`, `drive_owner` 등 |
| `pinky_observation["observed_at"]` | **Pinky 상태 시각**. 정렬 센서 측정 시각으로 복사하지 않음 |

센서는 `{pinky_id, pinky_boot_version, arrival_command_id, observed_at, alignment}` 기록을
반환합니다. 앞의 세 값은 **측정 당시의 관측**으로 묶고, `alignment`와 `observed_at`은
실제 정렬 센서의 판정과 측정 시각(timezone 포함 ISO 8601)으로 채웁니다. 오래된 측정에
새 boot/도착 ID 또는 현재 시각만 덧붙여 재사용하지 않습니다. 정차만으로 정렬을 확정하는
`virtual_alignment`는 mock 전용입니다.

콜백은 명령 접수 전과 실행 중에도 호출되므로, 장시간 기다리지 말고 해당 도착에 속한 최신
센서 기록을 반환합니다. 확인할 수 없으면 `None` 또는 `alignment: "unknown"/"unconfirmed"`로
반환합니다. 장치 읽기 실패도 가능한 한 `None`으로 반환합니다. 상태 발행 중 센서 예외가
발생하면 오류를 기록하고 `docking: null`로 발행하며 루프를 유지합니다. Host는 기본 3초
최신성·식별자 일치를 계속 검사하며, 접수 전에는 거절하고
작업 중 확인이 끊기면 정지 요청·HOLD로 처리합니다. MQTT 메시지나 Pinky 코드는 변경하지 않습니다.

`WarehouseExecutor`와 `IndependentArmExecutor`에는 실제 Robot/Policy/Vision 포트와
확정된 모델·캘리브레이션 버전이 필요합니다. 공동 `CooperativeShelfExecutor`는 실제 중지
콜백·ManagedPanelExecutor와, command/package/slot/arrival에 일치하는 최신 안착·해제 센서 콜백이 추가로
필요합니다. 패널의 `arm_return_only`/`shelf_align`만으로 재고 확정은 불가능합니다.
개별 `return_home`은 스키마에 예약된 동작이며 현재 Host는 항상 `unsupported_operation`으로 거절합니다.
홈 밖 복귀 경로는 아직 구현하지 않았고 모델/ArUco·정렬/안착 센서 바인딩도 현장 검증이 필요합니다.

검사: `bash ci/test.sh` 및 관제 서버의 `tests/test_omx_device_mqtt.py`,
`tests/test_omx_panel_integration.py`. 루프백 MQTT·실제 Host/Runner/PanelExecutor를
재사용하고 모터·정책·카메라/센서 입출력만 모사합니다. 현장 센서·모터·네트워크 성능은
별도 검증 대상입니다. AI UDP의 inference/request 상관관계 ID는 MQTT 최소화와 별개로 유지합니다.

### 정지 관측과 실물 적용 전 남은 조건

- 모든 실물 어댑터의 `stop_status()`는 모터 관측에서 `{"observed_at": "ISO-8601 UTC", "stopped": true/false}`를 반환합니다. 공동 작업은 양팔 모두 정지한 값이어야 합니다. 어댑터 PC와 같은 시각 기준의 관측만 받으며 허용 나이는 0~3초입니다. 정지 완료는 **해당 abort 요청 이후** 관측이어야 합니다. 콜백은 최신 센서 캐시를 짧게 읽고, 홈 위치·HTTP 성공·명령 접수를 정지 근거로 만들지 않습니다. 누락·오류·오래된 값은 준비/정지 확인으로 인정하지 않습니다.
- `WarehouseExecutor`는 진행 중 Runner에 취소를 전달하고, 재시작 등 Runner가 없으면 기존 Robot 포트의 `hold()`를 호출합니다. `IndependentArmExecutor`도 같은 경로를 사용합니다. `CooperativeShelfExecutor`는 현장 `stop` 콜백을 호출합니다. 관측이 없으면 `physical_stop_not_confirmed` + HOLD이며, 정지 관측 연결 전에는 실물 준비 상태가 되지 않습니다.
- v2 `CooperativeShelfExecutor`는 준비 조회·시작·결과 조회를 MQTT owner 밖에서 직렬 실행하고, 정지는 별도 작업자로 전달합니다. 모델 목록은 15초부터 갱신하고 기존 캐시는 최대 30초까지 허용합니다. 초기 준비가 끝나기 전에는 준비 완료로 표시하지 않습니다. 팔 준비도 먼저 조회하며, owner는 캐시만 읽고, 실제 POST 직전에는 worker가 팔 준비·정지·만료를 다시 확인합니다. 읽기 시작 시각부터 3초가 지난 관측은 준비/완료 근거로 재발행하지 않습니다. `stop_status`와 `completion`은 현장의 최신 센서 캐시를 짧게 읽는 콜백이어야 합니다. 작업 중 패널이 생략하는 팔 관측은 미확인으로 표시하며, 실제 조회 실패/장치 오류와 구분합니다. 생략된 관측으로 준비·정지·복귀를 확정하지 않습니다. v1 `--executor panel`의 직접 HTTP 경로는 그대로입니다.
- 정지/종료가 대기 중인 시작보다 먼저 처리되면 POST를 보내지 않습니다. 이미 POST 중이면 같은 command_id의 취소를 독립 경로로 요청합니다. 취소 기록과 실제 정지 관측이 확인되면 시작 HTTP 응답을 기다리지 않고 정지 완료를 보고합니다. 늦은 POST는 기록된 취소로 거절됩니다. 시작 응답이 유실되면 `panel_start_unconfirmed`로 HOLD하며, 정지 센서가 true여도 미확인 시작을 정지 완료로 덮어쓰지 않습니다. 자동 재시작·재전송은 없습니다.
- **패널 기반 v2 실물 factory는 아직 운영에 연결하지 않습니다.** 기존 패널 API는 클라이언트 timeout 뒤 늦은 시작을 취소한다고 보장하지 않습니다. 로컬 지연 시험에서도 이 경우를 재현했으며, 스레드 분리는 원격 취소 보장이 아닙니다. 아래 임시 HTTP 계약과 송수신 모듈을 준비했습니다. 외부 패널 담당자가 실제 프로세스 시작 경계에 수신부를 붙여야 하며, 현재 공개 패널에 이 API가 구현됐다는 의미가 아닙니다. MQTT 필드와 실물 factory의 장치 경계는 유지했습니다. 센서·모터·실물 캘리브레이션도 별도 검증 대상입니다.
- OMX의 새 이송 명령은 Pinky와 동일하게 `sent_at` 나이 −0.5~3초를 검사하고 벗어나면 `stale_message`로 거절합니다. `robot_stop_command`는 지연된 정지를 거절하지 않도록 `sent_at` 신선도만 제외하고 만료·현재 boot·정지 대상 명령/로봇 일치는 검사합니다. 기록된 명령의 동일 본문 재전송은 신선도 검사 전에 저장 결과를 반환합니다. 만료는 수신 PC의 `expires_at` 절대시각 기준입니다. 유지 확인은 `sent_at`의 나이 −1~1초, Pinky/정렬 관측은 −1~3초, 로컬 정지 관측은 0~3초를 검사합니다. 명령과 센서의 허용 창이 다르므로 관제·로봇 NTP와 센서 timestamp 기준을 맞춰야 합니다.
- HOLD 자동 해제는 제공하지 않습니다. 관제가 요청한 정지 완료를 먼저 확인하고 실제 물품 제거·팔 복귀·유휴 상태를 정리한 다음, 같은 journal로 Host를 재시작하면서 `--reconcile-note "현장 확인 내용"`를 붙일 수 있습니다. 미완료 명령/정지가 남거나 장치가 정지·복귀·빈 그리퍼를 확인하지 못하면 실패합니다. 명령 이력/outbox를 지우지 않고 복구 기록만 남기며, MQTT 작업 재시작은 하지 않습니다. PR71 OMX3 Host의 RECONCILING도 기존 `clear_reconciling(key)` 절차로 먼저 정리해야 합니다. 관제 측 작업 재개는 별도 승인 절차이며 이 옵션으로 새 명령을 실행하지 않습니다. journal 삭제를 복구 방법으로 사용하지 않습니다.
- 같은 `command_id`로 다른 본문을 보내거나 스키마·출처·토픽이 틀린 패킷은 로그 후 거절합니다. 기존 명령의 결과를 덮어쓸 새 상태 이벤트는 만들지 않습니다. 관제는 유효한 상태/기존 결과를 조회해 확인합니다.

`stock-omx`는 창고 팔의 역할 명칭입니다. MQTT 토픽·JSON·journal 및 Python `omx3` 모듈 이름은 기존 연동을 위해 유지합니다. 식별자까지 바꾸는 작업은 관제·로봇·명세를 함께 전환하는 별도 변경입니다.

OMX3 소스(`src/roport_omx/omx3/`)와 `ci/test_omx3*.py`는 팀원의 `task/omx3`
병합본 `32b5b9e7184e09c3dfd97b2fd0847a67aaeb9887`와 동일합니다(PR #71·#89·#95·#91·#96 포함).
관제 연결 수정은 `field_executors.py`에서 관리합니다. 실행 중에는 Runner만 모터 버스를
읽으며, 상태 조회는 홈/그리퍼 자세를 미확인으로 둡니다. 같은 팔의 다른 경로도 이때
조회하지 않고, 실행 종료 후 다시 실제 자세를 읽습니다. `Robot.ready()`와 `stop_status()`는
연결 상태·관측 캐시만 읽어야 합니다. 정책은 `reset(marker_id)`와 추론 대기 시 `act() -> None` 계약을 사용합니다.

독립 정지는 OMX 패널에 적용하며 Pinky 정지 경로(유지 확인 3초·ROS guard 1초 lease)는 변경하지 않습니다. MQTT 규약은 관제 소유로 `services/roport-control-server/interfaces/mqtt`에서 관리합니다. 루트 WSS 규약은 서비스–관제 경계이므로 별도 위치를 유지합니다.

### 임시 command 단위 패널 HTTP 계약

실제 장비 실행 프로세스를 소유한 패널 한 곳에서 `PanelCommands`를 사용합니다. 기존 원격 `/api/eval/start`를 다른 서버에서 감싸는 프록시는 늦은 실행을 취소할 수 없으므로 지원 방식이 아닙니다. 수신 경계는 `src/roport_omx/panel_commands.py`, 선택형 클라이언트는 `managed_panel.py`이며 기존 `PanelExecutor`는 유지합니다. 외부 패널 팀의 API가 확정되면 이 경계만 교체하고 MQTT 명령 ID/장치 센서 계약은 유지합니다.

| HTTP | 입출력 / 연결할 메서드 |
| --- | --- |
| GET `/api/control/capabilities` | `{ "command_cancel": "durable" }`; 실제 아래 동작을 구현한 패널만 응답 |
| POST `/api/control/commands` | `{command_id, expires_at, payload}` → `ledger.submit(body)`; payload는 기존 eval 옵션, expires_at은 timezone 포함 ISO 시각 |
| GET `/api/control/commands/{command_id}` | `ledger.get(id)` → `{command_id, state, observation}`; 없음은 `not_found` |
| POST `/api/control/commands/{command_id}/cancel` | `{}` → `ledger.cancel(id, actual_stop)`; 아직 접수 전이어도 취소 tombstone을 저장 |

1. 패널 시작 시 **동일 SQLite 파일과 단일 장치 소유 프로세스**로 ledger를 엽니다. 받은 payload의 모델/장비 옵션은 기존 패널 검증을 그대로 거칩니다. 수신 크기 제한·인증은 패널 HTTP 경계 책임입니다.
2. 새 요청을 `submit`으로 저장한 뒤 실제 장비 프로세스 생성 직전에 `start(id, begin)`을 호출합니다. `begin(payload)`는 잠금 안에서 로컬 프로세스만 생성하고 즉시 반환해야 합니다. 모델 로딩·모터 연결은 생성된 작업자에서 수행하고 원격 큐에 위임하지 않습니다. 이 짧은 생성 경계를 잠금 밖으로 옮기면 취소 완료 뒤 늦은 시작이 가능하므로 잠금을 유지합니다. 만료를 다시 확인하고, 취소된 ID나 동일 ID 재전송으로 두 번 실행하지 않습니다. 다른 내용의 ID 재사용은 충돌로 응답합니다.
3. 실행 결과는 기존 센서 결과를 `finish(id, observation)`으로 저장합니다. 완료는 `state=finished`, 실패는 `state=failed`이고 arms 등 기존 결과 구조를 보존합니다. `completed/failed` 조회는 이 저장 관측을 반환합니다.
4. 취소 요청은 running을 stopping으로 저장하고 실제 stop을 호출합니다. 이후 `confirm_stopped(id, {observed_at, stopped})`에 **취소 이후 0~3초 이내 실제 모터 정지 관측**이 들어와야 cancelled가 됩니다. 재시작 후 unknown 명령을 다시 취소하면 정지 기준 시각도 새로 기록합니다. pending/미접수 ID는 즉시 cancelled로 보존합니다. 늦은 start/finish는 취소를 되돌리지 않습니다.
5. 재시작 시 미종료 명령은 unknown이며 자동 재실행하지 않습니다. 같은 ID 취소와 정지 관측을 완료해야 다음 작업을 받습니다. `CooperativeShelfExecutor`는 명령별 취소/조회를 제공하는 `ManagedPanelExecutor`가 필요하며 구형 client는 생성 단계에서 거절합니다. HTTP 취소가 지연되어도 현장 정지 콜백을 먼저 호출합니다. 원격 취소 확정과 로컬 정지 관측을 모두 만족해야 정지 완료입니다. Host 재시작 시 journal의 미종료 명령 ID를 복원해 같은 ID를 취소하며, 이전 시작을 재전송하지 않습니다. v1 `--executor panel` 시험 경로는 유지되지만 v2 현장 연결에 대신 사용할 수 없습니다.

시작 응답은 같은 command_id와 pending/running 상태를 확인합니다. 확정 거절(4xx·cancelled·expired)과 timeout/5xx 결과 불명을 구분하며, 잘못된 요청은 400, ID/점유 충돌은 409입니다. 취소 조회는 시작 I/O 잠금을 사용하지 않습니다. 로컬 정지 후 취소 POST는 1회 최대 1초, 총 3초 안에서 최대 3회 시도하고 실패 로그를 남깁니다. 확인하지 못한 정지는 HOLD를 유지합니다.

모의 HTTP/SQLite 시험은 `ci/test_panel_commands.py`, 지연·타임아웃·취소와 로컬 정지는 `ci/test_panel_async.py`입니다. 외부 패널 배포·공동 양팔 실구동·모터 정지 센서 연결은 이 테스트에 포함하지 않습니다. `shelf-01`은 공동작업 controller이며, 개별 팔 상태 확인은 OMX1/2 각각에 적용합니다.

### PR #66 기능 인수 범위

`cooperative_host.py --executor panel`, 슬롯별 Route/출력 필터/eval 옵션, 두 예제 설정,
모든 경로 모델의 실제 팔 관측, `end_on_return`, 작업 결과·일반 중지/비상 정지 구분을 유지합니다.
관련 패널 회귀 테스트도 함께 유지하며 만료·미복귀 판정만 위 기준으로 수정했습니다.
별도 저장소의 제어판(3.12.0 이상) 설치·자동 Host 시작 설정과 현장 정책/캘리브레이션은
이 저장소의 PR 병합만으로 설치되지 않습니다. 구형 v1 시험과 v2 현장 규약을 혼용하지 않습니다.

## 현장 적용 준비

`panel_command_api.PanelCommandAPI`는 실제 장치 소유 프로세스의 HTTP 라우터에 설치해야 합니다. 현장 factory·센서 연결과 배포 매뉴얼은 후속 범위입니다.

Host client ID는 `roport-field-<정렬한 target 목록>`입니다(예: `roport-field-omx3`, `roport-field-omx1-omx2-shelf-01`). 저장소의 브로커 설정에는 이전 `roport-field-omx` 고정 ACL이 없고, 전송은 MQTT v5 `clean_start=True`, `SessionExpiryInterval=0`이므로 이전 persistent session을 인계하지 않습니다. 재전송은 로컬 journal/outbox가 담당합니다. 외부 브로커에 client ID 기반 ACL을 별도로 설정한 현장은 배포 전에 새 ID의 기존 cmd/ack/event 권한을 등록해야 합니다. 외부 브로커 설정은 이 PR에서 검사하지 않았습니다.

## shelf-01 협업 Host 연결 (시험 단계, FFT1-153)

OMX1·2 COOPERATIVE 작업 명령(`roport-omx-task-v1`, `roport/v1/{site}/workcell/shelf-01/{cmd,event,ack}`)을 받아 실행부로 넘기고 event를 회신합니다. MQTT 송수신은 관제 서버 `adapters/mqtt`를 `src/roport_omx/mqtt_link/`로 복사해 사용합니다(원본 커밋은 각 파일 첫 줄). 기본값은 모의 실행부이며, `--executor panel`을 지정한 경우에만 로컬 패널을 호출합니다.

```bash
# Host (journal은 재시작 후에도 남는 경로, /tmp 금지)
uv run python scripts/cooperative_host.py --protocol v1 --journal ~/.local/state/roport/test-shelf-01.json
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
- 호스트 메트릭(표시 전용): `--host-metrics-period-s 5`처럼 주기를 줄 때만 `roport/workcell/shelf-01/host`에 v2 `host_metrics`(QoS0, retain=false, boot_version=boot_id)를 발행합니다. 생략하면 꺼지며 기본값은 없습니다. 작업·ACK·HOLD·정지 판단에 쓰지 않고 발행 실패는 로그만 남깁니다. 실장비·실브로커에서는 아직 검증하지 않았습니다.
- 검사: `bash ci/test.sh` (`ci/test_cooperative.py`, `ci/test_panel_executor.py`, `ci/test_host_metrics.py`; 브로커·제어판·장비 불필요). 실제 브로커 왕복은 로컬 Mosquitto에서만 확인했고, 패널 연동·실물 팔은 미검증입니다.

## AI 추론 UDP 클라이언트 (시험 단계, FFT1-154)

`src/roport_omx/infer_link/`는 Host PC가 GPU AI 추론 서버(`services/roport-ai-server`의 `omx_udp`)와 UDP 규약 `RPI1`로 통신하는 클라이언트입니다. 규약 파일 `wire.py`는 AI 서버 쪽 원본을 그대로 복사하며 두 곳을 함께 갱신합니다.

통신 구조: 관제 서버 ↔ Host PC는 MQTT, Host PC ↔ AI 추론 서버는 UDP, Host PC ↔ OMX 팔은 USB 시리얼(Dynamixel Protocol 2.0)입니다. Host PC 안에서 관제 Host(`cooperative_host.py`)는 로봇 루프를 가진 OMX 제어판을 로컬 HTTP로 호출하고, 제어판이 이 클라이언트로 추론을 요청합니다. 현장에서는 제어판(3.13.0 이상)이 같은 코드의 복사본을 사용하며 추론 통신은 UDP만 씁니다.

- 관측 한 묶음(관절 값 + 카메라 JPEG + 작업 정보)을 한 메시지로 보내고, 모든 조각이 모여 CRC·HMAC을 통과한 메시지만 씁니다.
- `infer()`는 요청마다 새 `observation_seq`와 `schema_version·boot_id·inference_id·captured_at·deadline`, 실행 중인 MQTT 명령의 `task_id·command_id·execution_id`를 붙입니다. 응답이 같은 시도·모델을 되돌려주지 않으면 `RESULT_MISMATCH`·`MODEL_MISMATCH`로 거절해 모터 목표로 쓰지 않습니다.
- 응답이 시간 안에 없으면 `InferenceTimeout`이며 스스로 재시도하지 않습니다. 호출자는 최신 관측으로 다시 요청하고 그동안 팔을 안전하게 유지합니다.
- 한 번에 요청 하나만 보냅니다. `InferenceClient`는 소켓 하나로 모든 응답을 받으므로 스레드 사이에 공유하면 안 됩니다. 두 호출이 겹치면 한쪽 응답을 다른 쪽이 `stale_reply`로 버리고 원래 호출은 `InferenceTimeout`이 됩니다. 겹칠 수 있는 호출자는 직접 직렬화하거나 스레드마다 클라이언트를 따로 만듭니다.
- 서버 주소는 IP나 호스트 이름 모두 됩니다(`("192.0.2.10", 5600)`, `("gpu-host", 5600)`, JSON의 `["…", 5600]`). 생성할 때 IPv4로 한 번 바꿔 두고, 그 주소에서 온 응답만 받습니다.
- 공유 키 파일(hex, `chmod 600`)은 저장소에 올리지 않습니다.

### 새 Host PC를 GPU 추론 서버에 붙이기

GPU 추론 서버 하나를 여러 Host PC(예: OMX1·2 Host, OMX3 Host)가 함께 씁니다. 새 Host PC는 아래 세 가지가 갖춰져야 응답을 받습니다. 하나라도 빠지면 서버가 요청을 조용히 버리므로 호출 쪽에는 `InferenceTimeout`만 보입니다.

1. **같은 현장망**: GPU PC와 같은 공유기·같은 Wi-Fi(또는 유선)에 붙습니다. 게스트 Wi-Fi(기기 간 격리), 휴대폰 핫스팟, VPN 주소로는 닿지 않습니다. 서버는 허용 IP로 받으므로 공유기에서 이 PC의 IP를 고정(DHCP 예약)합니다. 가능하면 유선을 씁니다: 요청 하나가 datagram 수십 개로 나뉘고, 하나만 빠져도 그 요청 전체가 버려집니다.
2. **공유 키**: GPU PC 서버와 같은 키 파일이 필요합니다. GPU PC 담당자에게서 직접 받아 `~/.config/roport/omx-udp.key`에 두고 `chmod 600` 합니다. 키 내용은 메신저·이슈·PR·저장소에 붙이지 않습니다. 키가 다르면 응답이 `bad_mac`으로 버려집니다(`client.stats`).
3. **허용 IP 등록**: 고정한 IP를 GPU PC 담당자(GPU 제어판)에게 알려 서버 허용 목록에 넣습니다. 등록 전에는 요청이 `untrusted_peer`로 버려집니다.

확인 순서: 이 PC에서 `ip -4 -br addr`로 현장망 주소인지 보고, `ping <GPU PC IP>`가 되면 `InferenceClient((<GPU PC IP>, 5600), key).ping()`이 `{"ok": true, ...}`를 돌려주는지 봅니다.

여러 Host가 한 서버를 쓸 때 주의:
- 서버는 요청을 하나씩 처리합니다. 한 Host가 `load()`로 모델을 올리는 동안(수 초~수십 초) 다른 Host의 `infer()`는 응답이 늦어 시간 초과가 날 수 있습니다. 모델은 주행 전에 미리 올리고, 다른 Host가 주행 중일 때는 새로 올리지 않습니다.
- 서버가 동시에 올려 두는 모델 수에는 한도가 있습니다. 여러 Host가 서로 다른 모델을 번갈아 쓰면 매번 다시 올리게 되므로, 함께 쓸 모델 수를 GPU PC 담당자와 맞춥니다.
- Host와 세션별로 따로 조립·처리하므로 한 Host의 밀린 요청이 다른 Host의 요청을 밀어내지는 않습니다.
- 검사: `ci/test_infer_link.py`(loopback 가짜 서버, `wire.py` 복사본이 AI 서버 원본과 같은지도 확인). 제어판 연결, 실제 망 손실·지연, GPU 추론, 실물 팔은 이 검사에 포함하지 않습니다. 규약은 팀 AI UDP 설계안(Confluence MFS 10125435)의 원칙을 따르며 팀 합의 전 초안입니다.

### INFER 요청·응답 형식 (`schema_version` `roport-omx-infer-v1`)

서버는 OMX 제어판의 추론 형식을 그대로 받습니다. 제어판이 아닌 호출자(예: OMX3 `ChunkPolicy` 어댑터)는 아래대로 `infer(header, data)`의 두 인자를 만듭니다. `schema_version`, `inference_id`, `observation_seq` 같은 식별자는 `infer()`가 붙이므로 넣지 않습니다.

요청 `header`:

| 키 | 형식 | 뜻 |
|---|---|---|
| `model` | str | GPU PC의 체크포인트 폴더 경로(`pretrained_model`). 팀 checkpoints 아래만 허용되고, 아니면 `MODEL_NOT_ALLOWED`입니다. 먼저 `load()`로 올립니다. |
| `overrides` | dict | `load()`에 준 값과 같게 둡니다. 다르면 서버가 모델을 따로 올립니다. 없으면 `{}`입니다. |
| `task`, `robot_type` | str \| null | LeRobot 정책에 그대로 넘기는 작업 문장과 로봇 종류입니다(언어 조건 모델만 `task`를 씀). |
| `obs` | list | 관측 묶음입니다. 오래된 것부터 넣고 마지막이 최신입니다. `LOADED`의 `n_obs`개를 보냅니다(diffusion은 마지막 2개, 나머지 정책은 마지막 1개만 씀). |
| `chunk` | int | 돌려받을 동작 수의 상한입니다. 없으면 50입니다. |
| `rtc` | dict \| null | `{"delay": int, "left": [[…], …] \| null}`. RTC를 켠 smolvla·pi0·pi05 모델만 씁니다. `delay`는 추론 지연(제어 step 수), `left`는 이전 chunk에서 아직 실행하지 않은 동작의 `original` 값입니다. 그 밖의 모델은 무시합니다. |

`obs`의 원소 하나와 `data`:

```json
{"state": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
 "images": {"observation.images.front": [0, 18234], "observation.images.wrist": [18234, 17012]}}
```

- `state`: 한 시점의 `observation.state`(float). 순서·단위는 모델이 학습한 데이터셋과 같아야 합니다(데이터셋 `meta/info.json`의 `observation.state` `names`).
- `images`: 카메라 키 → `[offset, length]`. 키는 모델 `config.json`의 `input_features` 이미지 키와 같아야 합니다(`observation.images.<카메라 이름>`). `offset`·`length`는 `data` 안의 바이트 위치입니다.
- `data`: 모든 JPEG를 `obs` 순서, 그 안의 이미지 순서대로 이어 붙인 바이트입니다. 각 JPEG는 RGB 카메라 프레임을 OpenCV로 인코딩한 것입니다(BGR로 바꿔 `cv2.imencode`, 제어판 기본 품질 90). `LOADED`에 `image_size: [h, w]`가 있으면 그 크기로 `cv2.INTER_AREA` 축소 뒤 인코딩합니다(제어판 `omx_tools/omx_infer.py`의 `pack_observations`와 같음). 서버는 `cv2.imdecode` 뒤 RGB로 되돌립니다.

응답(`infer()`의 반환값):

| 키 | 뜻 |
|---|---|
| `actions` | `[[…], …]` 길이 ≤ `chunk`, 행마다 모델의 action 차원(학습 데이터 `action`과 같은 순서·단위, 정규화를 푼 값). 로봇 목표값으로 쓰는 값입니다. |
| `original` | 같은 모양의 정책 원래 출력(정규화된 값)입니다. RTC의 `left`로만 다시 씁니다. |
| `infer_ms`, `load_ms` | GPU 추론 시간, 이 요청 때문에 모델을 올린 시간(이미 올라가 있으면 0)입니다. |
| `model_version`, `valid_until` | 요청 모델(또는 `model_version`), 요청의 `deadline`입니다. |
| 식별자 | `inference_id·observation_seq·task_id·command_id·execution_id`를 되돌려주며, `infer()`가 요청과 비교합니다. |

동작 값이 NaN·Infinity면 서버가 `INFER_FAILED`로 거절하고, 규약도 유한하지 않은 수를 싣지 않습니다.

관절 값과 카메라는 모델이 학습한 데이터가 기준입니다(데이터셋 `meta/info.json`의 `names`, 모델 `config.json`의 `input_features`). 팀 데이터의 실제 예:

| | 한 팔 (`omx_follower`, 예: OMX3) | 양팔 (`bi_omx_follower`, 예: OMX1·2) |
|---|---|---|
| `state`·`actions` 한 행 | 6개: `shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper` | 12개: `left_` 6개 다음 `right_` 6개(같은 관절 순서) |
| 단위 | 몸통 관절 −100~100, 그리퍼 0~100 | 같음 |
| `images` 키 | 모델에 따라 다름. 예: 정면만 쓰는 ACT는 `observation.images.front` 하나 | 예: `front`, `front_2`, `wrist_left`, `wrist_right` |

- 단위는 LeRobot `omx_follower` 기본값(`use_degrees=False`)입니다. 각도가 아니라 **팔마다의 캘리브레이션 파일** 범위로 정규화한 값이므로, 같은 값도 팔이 다르면 실제 자세가 다를 수 있습니다. 모델은 그 데이터를 기록한 팔과 같은 `robot_id`·캘리브레이션으로 씁니다.
- 모델이 받지 않는 카메라는 넣지 않아도 됩니다. 모델이 받는 카메라가 빠지면 서버가 `INFER_FAILED`로 답합니다.
- `chunk`는 모델이 한 번에 만드는 수(`config.json`의 `chunk_size`, 예: 100)보다 크게 받을 수 없습니다. 빼면 50으로 잘리므로 필요한 값을 직접 넣습니다.

`omx3/adapters/udp_infer.py`의 `UdpInferClient`는 OMX3 `ChunkPolicy`에 주입할 추론 구현입니다. `load()` 후 정면 RGB 영상 1장과 관절 상태 6개를 보내며, 이미지 크기·JPEG 품질·동작 순서는 위 규약을 따릅니다. 시간 초과는 기존 `ChunkPolicy`의 새 관측 재요청으로 처리합니다. 공통 `infer_link`는 표준 라이브러리만 사용하고, 이 어댑터는 장비용 OpenCV 환경에서 사용합니다.

이 동기화는 #109·#113·#115의 모듈과 검사를 포함합니다. 실물 Host 시작점에 어댑터를 주입하는 작업, N01 도킹 보정과 상자 안착 확인, 실제 모델·캘리브레이션 버전 값은 별도 확인이 필요합니다. N01 별도 정렬 센서는 요구하지 않습니다. 모듈 추가만으로 GPU 연결이나 팔 구동이 자동 시작되지는 않습니다.

### 선반 B열의 Fleet 연동

`scripts/cooperative_host.py`의 기본 실행은 `roport-control-v2`다. 선반 작업셀과 양팔의 MQTT 응답은 이 프로세스에서 발행한다. 같은 엔드포인트를 발행하는 다른 Host를 함께 실행하지 않는다.

가상 Fleet과 연결할 때 모의 팔을 사용하고 경로에 기대하는 모델·보정 버전을 맞춘다.

```bash
uv run python scripts/cooperative_host.py --executor mock \
  --mock-versions sim-model sim-cal --journal ~/.local/state/roport/shelf-v2.json
```

실물 v2 제어판 설정은 `config/shelf_routes.example.json`을 기준으로 한다. 실물 이송은 해당 Pinky의 자체 상태(8초 이내 관측, N11 정차, 도착 ID·boot 일치)로 방문을 확인한다. 작업셀 도킹 센서 확인(`docking_alignment_unconfirmed`)은 쓰지 않으며(2026-11-23 결정, #154) `robot_state`의 `docking.alignment`는 `unknown`이다. 실물 정지 표본 확인은 현장 적용 단계다.

`task/omx-cooperative` #156·#157·#162·#163·#169·#171과 같은 동작이다(`cooperative_v2.py`는 같은 파일, `shelf_panel.py`는 그 `panel_executor.py` 사본).

- 신선도 기준은 관제와 같은 8초(`FRESH_S`): Pinky 상태, Host 상태 표본, 명령 `sent_at`, 작업 유지 확인(watchdog).
- 입고 놓였음 즉시 완료(`--report-placement`, v2 기본 켜짐, `--no-report-placement`로 끔): 놓음 판정이 상자가 명령 칸에 놓이고 두 손이 빠진 것을 보면 `completed`(`evidence: "shelf_placed"`)를 보내고, 정렬이 끝나면 같은 명령으로 `completed`(`shelf_align`/`shelf_load_not_aligned`) 또는 `failed`+HOLD를 다시 보낸다(관제 #158). 정렬 중에도 watchdog은 유지하고, Pinky 이탈 감시는 관제가 놓였음 보고를 저장할 때까지 유지한다.
- 출고 쟁반 놓음 즉시 완료(`--report-unload-placement`, 기본 꺼짐, 제어판 3.24.1 `tray_judge` on): 상자가 Pinky 쟁반에 놓이고 두 손이 빠진 것을 보면 `completed`(`evidence: "pinky_placed"`)를 먼저 보내고, 양팔이 홈에 오면 `completed`(`pinky_load`)를 다시 보낸다. 관제 Fleet이 출고 첫 보고를 저장·ACK해야 켤 수 있어 기본은 지금처럼 한 번만 보고한다(#169). 홈 복귀 중 손끝이 쟁반 구역에 다시 들어와 5초 안에 못 빼면 `PANEL_TRAY_REENTRY_HOLD`로 실패·HOLD.
- 한팔 이송(#171, 제어판 3.25.0): `omx1`·`omx2` 토픽의 `arm_action_command`(`operation: transfer`)를 받는다. 칸 열이 그 팔과 맞아야 하고(A = omx1, C = omx2, 아니면 `slot_arm_mismatch`), Pinky는 그 열의 정차 노드(A N21, C N22)에 있어야 한다. 경로는 `(작업, 칸, DEDICATED)`이며, 팔이 홈에 온 뒤 `completed`를 한 번 보낸다(입고 `shelf_load_one_arm`, 출고 `pinky_load`). `return_home`은 `unsupported_operation`으로 거절한다. 한팔 경로가 없으면 지금과 같다.
- 제어판 `/api/eval/start`(수 초)는 MQTT 반복문 밖 스레드에서 기다리고, 접수 검사는 상태 표본 스레드의 값을 쓴다. 기다리는 동안에도 Pinky 상태·작업 유지 확인을 받고 `robot_state`를 보낸다(2026-11-23 현장 첫 v2 입고의 HOLD 원인). 작업 중에는 `robot_state`에 fault를 싣지 않는다.
- 제어판 학습 모델 목록을 아직 한 번도 받지 못했으면(Host 시작 직후, 제어판이 늦게 켜짐) 이송 접수를 보류하고 목록은 뒤에서 받는다. 3초 안에 준비되지 않으면 `rejected` `executor_state_stale`이며, 관제는 이 거절을 HOLD로 처리하므로 운영자 복구가 필요하다(#162).
- 명령마다 journal `timing`에 수신·접수·시작·제어판 단계·종료 시각을 남기고 Host 로그에 `timing <명령> <구간> +초`로 찍는다.
- 상자 떨어뜨림 가능성(출고 `PANEL_BOX_DROPPED`, 입고 `PANEL_BOX_NOT_ON_SHELF`)은 팔을 그 자리에 두고 HOLD. 제어판 3.22.0 손목 카메라 좌우 확인 실패(err 13)는 입고·출고 모두 `PANEL_WRIST_CHECK`(카메라별 변화량을 `detail`에)로 HOLD 없이 실패한다. 출고에서는 쟁반 확인보다 먼저 분류한다(#163). 기존 OmxTrial 시험 실행기는 `--protocol v1`과 `episode_s`가 있는 기존 경로 파일을 사용한다(`config/cooperative_routes.example.json`).
