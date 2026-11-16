# OMX 현장 적용 매뉴얼

기준: 2026-11-16 로컬 개발본 / MQTT `schema_version=2.0`, `profile=roport-control-v2`.
[공통 준비](README.md) 후 적용한다. MQTT Host·어댑터·패널 수신 모듈은 준비되어 있다.
**실제 패널의 시작 경계, 모터 정지·물품 안착 센서와 모델/캘리브레이션 연결은 현장 담당 코드가 필요하다.**
그 값이 없으면 내일 MQTT 진단까지 진행하고 상하차 완료를 더미로 꾸며 다음 단계로 넘기지 않는다.

## 1. 담당자가 준비할 정보

| 대상 | 실제 작업 / 필요한 값 |
| --- | --- |
| `omx3` | N01 창고 상차. 기존 Runner·로봇·정책·카메라, package→ArUco ID, package→small/large/unknown |
| `omx1` | N21의 1a/2a INBOUND·OUTBOUND 각각의 학습 정책·정렬/안착 영역 |
| `omx2` | N22의 1c/2c INBOUND·OUTBOUND 각각의 학습 정책·정렬/안착 영역 |
| `shelf-01` | N11의 1b/2b INBOUND·OUTBOUND 공동 정책, 양팔 정지·복귀·해제·물품 안착 관측 |
| 공통 | 실제 model_version/calibration_version, 브로커 주소, control_id, 같은 시각 기준, 영속 journal |

기체 자체의 IP는 명령 주소가 아니다. Host가 관제 브로커로 연결하고 target 토픽을 구독한다.
양팔 공동/개별 실행은 **같은 모터 소유자**를 재사용한다. OMX1/2를 소유하는 shelf Host는 하나로
구성하고 OMX3는 별도 PC/Host로 분리할 수 있다. Host의 MQTT client ID는 담당 target 목록으로
구분되어 서로를 끊지 않는다. 같은 팔을 여러 Host에서 동시에 소유하는 구성은 사용하지 않는다.

## 2. 코드 설치

관제 PC가 생성한 handoff 폴더를 OMX PC의 새 snapshot 폴더로 rsync하고 해시를 확인한다.
이미 같은 경로가 있다면 새 버전 폴더를 사용한다.

```bash
# 관제 PC
ssh OMX_USER@OMX_PC_IP 'mkdir -p ~/roport-field/snapshot'
rsync -av "$HANDOFF/" OMX_USER@OMX_PC_IP:~/roport-field/snapshot/
# OMX PC
cd ~/roport-field/snapshot
sha256sum -c SHA256SUMS
cp -a ~/roport-field/snapshot ~/roport-field/current
export FIELD_DIR="$HOME/roport-field/current"
source "$FIELD_DIR/broker.env"
```

실물 LeRobot/PyTorch/카메라 환경은 팀원이 검증한 기존 Python 3.12 환경을 사용한다.
배포 폴더에는 LeRobot 원본·정책 가중치·개인 캘리브레이션이 없다. 새 uv 환경만 만들어서
기존 GPU/장치 환경을 교체하지 않는다. 아래 `OMX_PYTHON`은 기존 환경의 Python 절대경로다.

```bash
export OMX_PYTHON=/absolute/path/to/existing-device-env/bin/python
export PYTHONPATH="$FIELD_DIR/omx/src:/absolute/path/to/site-adapters"
"$OMX_PYTHON" -c 'import roport_omx, paho.mqtt; print(roport_omx.__file__)'
```

paho-mqtt는 2.1.0 기준이다. 다른 버전이면 기존 의존성 영향 확인 후 고정한다.
읽기 전용 모터 버스 점검이 필요하면 **다른 버스 소유 프로세스를 종료한 후에만** 실행한다.
이 도구는 ping/register read만 하며 LeRobot.connect·토크·보정 쓰기를 호출하지 않는다.

```bash
"$OMX_PYTHON" "$FIELD_DIR/omx/scripts/check_hardware.py" --port /dev/serial/by-id/ACTUAL_DEVICE --output "$FIELD_DIR/omx-bus-check.json"
```

성공은 버스 읽기 성공이며 준비/정지/상하차 성공 판정이 아니다. 모터 모델·ID는 vendored OMX
설정(11~16) 기준이다. 다른 기체 구성이면 담당자가 설정을 확인해야 한다.

## 3. 기존 장치 실행부를 v2 factory에 연결

`field_host.py --factory module:function`은 `(executors, alignment_sensor)`를 받는다.
각 executor는 기존 어댑터를 사용한다. 다음은 **구성 형태**이며 변수들은 현장 포트로 채운다.

```python
from roport_omx.field_executors import (
    WarehouseExecutor, IndependentArmExecutor, PanelFieldExecutor,
)
from roport_omx.managed_panel import ManagedPanelExecutor

# OMX3 PC의 create_executors() 안:
warehouse = WarehouseExecutor(
    actual_warehouse_runner,
    marker_ids=package_to_marker,
    package_sizes=package_to_size,
    model_version=actual_model_version,
    calibration_version=actual_calibration_version,
    stop_status=read_latest_motor_stop,
)
# return {"omx3": warehouse}, read_latest_pinky_alignment

# 인도장 PC의 create_executors() 안:
left = IndependentArmExecutor(
    "omx1", calibrated_left_runners, marker_ids=package_to_marker,
    model_version=left_model_version, calibration_version=left_calibration_version,
    stop_status=read_latest_left_stop,
)
right = IndependentArmExecutor(
    "omx2", calibrated_right_runners, marker_ids=package_to_marker,
    model_version=right_model_version, calibration_version=right_calibration_version,
    stop_status=read_latest_right_stop,
)
shared = PanelFieldExecutor(
    ManagedPanelExecutor(cooperative_routes, box_marker=actual_box_marker),
    model_version=shared_model_version, calibration_version=shared_calibration_version,
    completion=read_transfer_evidence, stop=local_physical_stop,
    stop_status=read_latest_both_arm_stop,
)
# return {"omx1": left, "omx2": right, "shelf-01": shared}, read_latest_pinky_alignment
```

`calibrated_left_runners`는 `{("INBOUND","1a"): runner, ...}` 형식이다. 작업 칸·방향마다 실제
보정을 사용한다. 공동 `cooperative_routes`는 기존 `PanelExecutor.Route` 매핑을 유지한다.
패널의 실제 프로세스가 팔을 소유한다면 독립 작업도 그 소유자에 위임하는 현장 포트가 필요하다.
위 구조만 보고 같은 시리얼 포트를 Runner와 패널이 따로 열게 만들지 않는다.

| 콜백 | 제공할 실제 관측 |
| --- | --- |
| `stop_status()` | `{observed_at: ISO UTC, stopped: bool}`. 공동은 양팔 모두. 요청 이후 0~3초 내 정지 관측 |
| `alignment(command)` | 관측된 pinky_id·pinky_boot_version·arrival_command_id·observed_at·alignment. 명령 내용을 그대로 복사해 confirmed로 만들지 않음 |
| 공동 `completion(task)` | command/package/slot/arrival 연관성을 가진 최신 안착·해제 증거. 정확한 필드 형식은 아래 참고 |
| 실제 Runner | 팔 홈 복귀, 그리퍼 해제, 목적지 카메라 영역의 물품 안착 판정 |

공동 `completion(task)`의 반환 예시(값은 실제 센서 관측과 연결된 기록이어야 함):

```json
{
  "command_id": "observed-transfer-command",
  "package_id": "observed-package",
  "slot_id": "1b",
  "arrival_command_id": "observed-pinky-arrival-command",
  "observed_at": "2026-11-17T01:00:00+00:00",
  "placement_confirmed": true,
  "release_confirmed": true
}
```

미관측·오류·오래된 값은 준비 성공이 아니다. 콜백은 최신 센서 캐시를 빠르게 읽는다.
모델과 calibration 이름은 `fleet.json.omx`의 대응 target과 같아야 한다.
크기 카탈로그는 임시 운영자 입력이며 AI 추론 성공으로 표시하지 않는다.

## 4. 공동 작업 패널의 명령별 HTTP 수신부

현재 외부 패널의 기존 `/api/eval/start`만으로는 취소 뒤 늦은 실행을 막을 수 없다.
`PanelCommandAPI`와 `PanelCommands`를 **실제 로봇 실행 프로세스를 생성하는 패널 내부**에 붙인다.
원격 legacy API를 감싸는 별도 프록시로 대체하지 않는다.

```python
from roport_omx.panel_commands import PanelCommands
from roport_omx.panel_command_api import PanelCommandAPI

ledger = PanelCommands("/persistent/path/panel-commands.sqlite")
api = PanelCommandAPI(
    ledger,
    validate_payload=existing_eval_option_validation,
    begin=spawn_local_device_process,
    stop=local_physical_stop,
)
# 기존 HTTP 라우터에서:
# status, response = api.handle(method, path, parsed_json_body)
# response를 JSON으로 반환하고 HTTP status 적용
```

`begin(payload)`는 기존 armed·장비 준비 검사 후 **로컬** 프로세스를 생성하고 즉시 반환한다.
그 경계 뒤에 다시 원격 시작 큐를 두지 않는다. payload는 기존 모델/장비 옵션 검증을 그대로 거친다.
기존 수동 실행과 이 경로는 같은 장치 소유권/실행 잠금을 사용해야 한다.
패널 HTTP의 인증·본문 크기 제한은 기존 서버 책임이며 localhost에서 연결한다.

| 요청 | 처리 |
| --- | --- |
| GET `/api/control/capabilities` | `command_cancel: durable` |
| POST `/api/control/commands` | command_id·expires_at·payload 저장 후 만료/취소 검사, 단 한 번 시작 |
| GET `/api/control/commands/{id}` | 해당 명령의 상태·저장 관측 조회 |
| POST `/api/control/commands/{id}/cancel` + `{}` | 미접수 명령도 취소 이력 보존. 실행중이면 정지 요청 |

실제 완료 콜백은 `ledger.finish(command_id, observation)`에 기존 정상/실패·팔 관측을 전달한다.
`observation.state`는 `finished` 또는 `failed`다. 취소 후 모터 관측이 들어오면
`ledger.confirm_stopped(command_id, sample)`을 호출한다. HTTP 호출자가 완료 증거를 쓰는
API는 없다. `begin` 실패는 unknown, stop 실패는 stopping을 유지하며 자동 재실행하지 않는다.
재시작해도 같은 SQLite를 열고 미완료 명령을 현장 대조한다.

## 5. Host 실행과 현장 적용 순서

factory를 실제 모듈에 연결한 뒤 실행한다. 아래 예시는 그 모듈을 `field_devices.py`로 둔 경우다.
브로커 설정·control 이름·모델 버전을 먼저 맞추고 `--mock`을 실물 브로커에 사용하지 않는다.

```bash
cd "$FIELD_DIR/omx"
mkdir -p "$HOME/.local/state/roport"
# 인도장 PC: factory가 omx1/omx2/shelf-01을 반환
"$OMX_PYTHON" scripts/field_host.py --factory field_devices:create_executors --control control-01 --journal "$HOME/.local/state/roport/shelf-field-v2.json" --hardware-authorized
# 창고 PC: 그 PC의 factory는 omx3만 반환, journal은 별도
# "$OMX_PYTHON" scripts/field_host.py --factory field_devices:create_executors --control control-01 --journal "$HOME/.local/state/roport/warehouse-field-v2.json" --hardware-authorized
```

연결 직후 JSON의 ready는 MQTT 연결 의미다. 장치 준비는 `robot_state`로 판단한다.
관제의 `check_field.py --listen --targets omx3` 또는 `--targets omx1 omx2 shelf-01`로 확인한다.
실행부/UI 요청 처리는 계속 끈다. 패널 armed를 켜는 동작 자체가 장비 초기화를 한다면
현장 담당자가 안전 조건 확인 후 수행한다. 점검 도구는 자동으로 armed를 켜지 않는다.

정상 상태: active_command_id=null, fault=null, stopped/ready/home_reached=true,
holding_state=empty, holding_package_id=null. 공동 controller의 양팔도 각각 확인한다.
그 뒤 실제 Pinky 도착·정렬을 확인하며 단독 상하차 한 건부터 시험한다.
완료는 물품 안착·해제·팔 복귀·정지가 모두 맞을 때만 관제가 반영한다.

실패 시 원래 command_id·물품 위치·정지 관측을 보존한다. 현장 정리 후 동일 journal로
Host를 재실행할 때 `--reconcile-note "현장 확인 내용"`를 사용할 수 있으나 미완료 정지나
팔 관측이 맞지 않으면 해제되지 않는다. 관제 HOLD도 별도로 대조/해제하고 재승인한다.

현재 제한: 개별 `return_home` 명령은 예약 규격이며 구현되지 않았다. 실제 홈 밖 복귀는
현장 담당 절차로 수행한다. panel API 장착·센서·모델이 미완성이면 MQTT 연결까지만 성공이며
전체 실물 상하차 준비 완료로 보고하지 않는다.
