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
- 호스트 메트릭(표시 전용): `--host-metrics-period-s 5`처럼 주기를 줄 때만 `roport/workcell/shelf-01/host`에 v2 `host_metrics`(QoS0, retain=false, boot_version=boot_id)를 발행합니다. 생략하면 꺼지며 기본값은 없습니다. 작업·ACK·HOLD·정지 판단에 쓰지 않고 발행 실패는 로그만 남깁니다. 실장비·실브로커에서는 아직 검증하지 않았습니다.
- 실물 실행: `--executor panel --routes <경로 파일> --box-marker <상자 ID>`는 같은 PC의 OMX 제어판(v3.14.0 이상) API로 평가 1회와 이어지는 선반 정렬을 시작합니다(`shelf_align: true`). 칸마다 모델이 다르므로 `(operation, slot_id)`별 모델·작업 문장·시간을 경로 파일에 둡니다(`config/cooperative_routes.example.json`). 경로마다 제어판 출력 필터(`output_filter`: 약·중·강, 생략하면 끔)와 `filter_gripper`를 지정하면 관제로 시작한 평가에도 그대로 적용합니다. 경로가 없는 칸, 학습 목록에 없는 모델, 제어판 armed 꺼짐은 동작 전에 거절합니다. 완료는 제어판 `last_result`의 정상 종료·양팔 복귀·그리퍼 놓음(52+5% = 54.6 이하)으로 판정합니다. 제어판 3.12부터 Host는 `end_on_return`을 보내, 팔이 처음 자세에서 벗어났다가 돌아오고(모든 관절 8 이내) 그리퍼가 54.6 이하로 0.3초 유지되면 `episode_s` 전이라도 평가를 끝냅니다(정책의 재시도 방지). 조기 종료 신호(`last_result.last_episode.returned`)만으로 안전 복귀를 확정하지 않습니다. 완료·실패 구분은 아래 "선반 적재 미션 판정"을 따릅니다. 제어판 3.9부터 평가 뒤 선반 정렬이 이어지면 `final`이 될 때까지 기다리고, 정렬 성공·정렬 뒤 쉬는 자세 확인(`align.rest`)·최종 `/api/arms` 홈/그리퍼 실측을 함께 판정합니다(`evidence: "shelf_align"`). 사람 조치를 기다리는 `hold`(비상 정지, 내려놓기 경로 없음)는 기다리지 않고 바로 실패·HOLD로 보고합니다. 제어판에서 사람이 누른 중지·즉시 중단은 정지가 끝난 뒤 `PANEL_STOPPED`·`PANEL_ABORTED`(정렬 중이면 `PANEL_ALIGN_STOPPED`)로 실패 보고하되 HOLD하지 않습니다. 정지하면 제어판 armed가 꺼지므로 다음 시작은 사람이 armed를 다시 켜야 합니다. 비상 정지·강제 종료·내려놓기 실패·오류·미복귀는 HOLD합니다. 상태 조회(`status.requested`)의 양팔 상태는 제어판 3.11의 `GET /api/arms?model=…`(실행 중이 아닐 때 모터 위치를 읽음)에서 가져오며, 경로표의 모든 모델 홈 자세 안이어야 `returned`입니다. 그리퍼는 54.6 이하면 놓음, 넘으면 `grasping`으로 봅니다(2026-11-10 시험 합의: 빈 채로 닫히면 48~51, 상자에 막히면 58~60, 열림 61~67. 상자 모서리를 50 근처에서 쥘 수도 있어 놓음 여부는 팔 복귀와 함께 판정). 제어판 3.11부터 armed를 켤 때 `~/.config/roport/omx-host.json` 설정(예: `config/omx-host.example.json`, `link_config`는 `execution_mode: hardware`여야 함)으로 이 Host를 자동 시작하고, armed를 꺼도 Host는 남아 보고·상태 조회에 답합니다. Host는 제어판 **3.14.0 이상**이고 AI 추론 UDP 연결이 살아 있을 때만 준비 상태로 보고합니다(아래 "AI 추론 연결 확인"). **팔이 움직이므로 사람이 옆에서 armed를 켠 상태로만 시험합니다.** 상자 ID는 시험용 고정값이며 `package_id` 규칙은 관제와 합의 후 연결합니다.
- 준비 조회·모델 검사 이후, 접수 journal 저장 이후와 `/api/eval/start` POST 직전에 각각 `expires_at`을 재검사합니다. 만료되면 새 평가를 시작하지 않습니다. HTTP 시작 응답이 끊기면 결과 불명·HOLD이므로 자동 재시작하지 않습니다.
- 검사: `bash ci/test.sh` (`ci/test_cooperative.py`·`ci/test_host_metrics.py`·`ci/test_panel_executor.py`; 브로커·제어판·장비 불필요). 실제 브로커 왕복은 로컬 Mosquitto에서만 확인했고, 패널 연동·실물 팔은 미검증입니다.

### 선반 적재 미션 판정 (2026-11-16 합의)

이 실행부는 선반 적재(`shelf_inbound`)만 판정합니다. 하차(`shelf_outbound`)는 정렬 없이 Pinky 쟁반 작업 전후 비교와 팔 복귀로 판정하는 별도 기능에서 받으며, 그 전에는 팔을 움직이지 않고 `UNSUPPORTED_OPERATION`으로 거절합니다(정렬 요청·적재 판정이 하차에 적용되지 않게).

선반 적재 미션은 적재 뒤 상자 정렬까지 포함합니다. Host는 시작 요청에 `shelf_align: true`를 보내고, 평가·정렬 결과(`last_result.align`)와 양팔 실측으로 판정해 이벤트 payload의 `mission_result`로 보고합니다.

| `mission_result` | 조건 | 이벤트 | HOLD |
| --- | --- | --- | --- |
| `MISSION_COMPLETE` | 정렬 성공(명령한 칸), 양팔 홈·그리퍼 놓음 | `completed`, `evidence: "shelf_align"` | 없음 |
| `MISSION_COMPLETE_NOT_ALIGNED` | 상자가 선반 위(`on_shelf: true`), 정렬은 시간 초과·실패·건너뜀, 양팔 홈·그리퍼 놓음 | `completed`, `evidence: "shelf_load_not_aligned"`, `align_error` | 없음 |
| `MISSION_FAILED` | 평가 오류, 상자가 선반 위에 없음(`PANEL_BOX_NOT_ON_SHELF`), 다른 칸 정렬(`PANEL_ALIGN_WRONG_SLOT`), 팔 미복귀·그리퍼 쥠(`ARM_NOT_RETURNED`), 작업자 중지·즉시 중단 | `failed` | 없음: 제어판 `/api/home/return`으로 양팔을 평가 시작 자세로 보낸 뒤 `/api/arms`로 홈·그리퍼 놓음을 확인하고 보고 |
| `MISSION_HOLD` | 선반 판독 불가(`on_shelf: null` → `PANEL_SHELF_UNREADABLE`, 정렬·판독 모두 없음 → `PANEL_ALIGN_SKIPPED`·`PANEL_ALIGN_NOT_RUN`), 비상 정지·강제 종료·내려놓기 실패, 결과 불명, 홈 복귀 거절·시간 초과·복귀 후에도 미복귀 | `failed` | 있음: 현장 확인 후 `--clear-hold` |

- 양팔 홈 판정: 정렬이 돌았으면 정렬 뒤 쉬는 자세 확인(`align.rest`), 돌지 않았으면 평가 결과의 홈 판정을 쓰고, 완료·실패 보고 전에 `/api/arms`로 최종 모터 위치·그리퍼를 다시 읽습니다. 이미 홈이면 이동 요청을 보내지 않습니다.
- 홈 복귀는 그리퍼를 열기 때문에, 쥐고 있던 상자는 놓습니다. 비상 정지로 멈춘 팔은 자동으로 움직이지 않습니다.
- 제어판은 PC 설정(`shelf_align.slot`, 현재 2B 하나)의 칸에서 정렬합니다. 다른 칸 미션은 `PANEL_ALIGN_WRONG_SLOT`으로 실패하므로, 칸별 정렬은 제어판이 명령의 칸을 받도록 바뀐 뒤 사용합니다.
- 관제(`omx_trial`)는 `arm_return_only`(모의 실행부)·`shelf_align`·`shelf_load_not_aligned`만 완료로 받고, Not Aligned 사유를 이송 기록의 `align_error`에 남깁니다. 관제는 현재 모든 `failed`를 HOLD로 처리합니다(`MISSION_FAILED` 후 관제 흐름은 관제 담당과 합의 필요).

### 하차 미션 판정 (2026-11-17 합의)

하차는 선반의 상품을 관제 명령으로 대기 중인 Pinky 쟁반에 상차하는 작업입니다(`shelf_outbound.requested`, 예: 2b 칸 모델 `outbound_act_re1112x37_60k_1113`). Host는 시작 요청에 `pinky_check: true`, `shelf_align: false`, `draw_target: false`를 보내고, 평가 뒤 제어판이 카메라만으로 확인한 결과(`last_result.pinky`)로 판정합니다. 시간 기준은 쓰지 않습니다.

- 칸 비었음(`slot_empty`): 선반 정면 카메라(`front_2`)로 명령한 칸에 상자가 남았는지 읽습니다(빈 선반 기준 사진과 비교).
- 상차됨(`on_pinky`): 위 카메라(`front`)로 다음 중 하나면 `true`입니다.
  1. 상자 윗면 마커(15~22)가 쟁반 영역 안에서 읽힘(`box_on_tray`, 기본)
  2. 쟁반이 그 Pinky의 빈 쟁반 기준 사진과 달라짐(`tray_changed` ≥ 0.25, 보조). 상자가 기울거나 과노출로 마커가 안 읽힐 때를 대비합니다.
- **쟁반에는 마커를 붙이지 않습니다.** 하차 정책은 마커 없는 쟁반으로 학습되어, 쟁반에 마커가 있으면 동작을 시작하지 않습니다.
- 빈 쟁반 기준 사진은 Pinky마다 한 번 찍습니다(정렬 실행기 `--step tray_check --pinky N --tray-ref-reset`). 기준 사진이 없으면 `on_pinky: null`(HOLD)입니다.
- 확인은 정렬 실행기 `--step pinky_read`가 하며 로봇을 움직이지 않습니다. 제어판은 비상 정지·강제 종료가 아니면 평가가 어떻게 끝났든 확인합니다.
- 하차 평가에서는 위 카메라 관측에 목표 원을 그리지 않습니다(`draw_target: false`, 제어판 3.16.1+). 목표 원 없이 학습된 모델이라, 그리면 동작이 크게 느려집니다.

| `mission_result` | 조건 | 비고 |
| --- | --- | --- |
| `MISSION_COMPLETE` | `slot_empty: true`, `on_pinky: true`, 양팔 홈·그리퍼 놓음(확인 뒤 `/api/arms` 실측) | `evidence: "pinky_load"` |
| `MISSION_FAILED` | 칸에 상자가 남음(`PANEL_BOX_STILL_ON_SHELF`) 또는 칸은 비었는데 쟁반이 빔(`PANEL_BOX_DROPPED`) | 양팔 홈 복귀 후 HOLD 없음 |
| `MISSION_HOLD` | 칸 판독 불가(`PANEL_SLOT_UNREADABLE`), 쟁반 판독 불가(`PANEL_PINKY_UNREADABLE`), 확인 없음(`PANEL_PINKY_CHECK_NOT_RUN`), 칸과 쟁반 둘 다 상자(`PANEL_PINKY_CHECK_CONFLICT`), 비상 정지 | 현장 확인 후 HOLD 해제 |

현장 시험(2026-11-17, 제어판 3.17.0, 비동기 100 / 0.2 / latest_only, 필터 중): 정책 성공 6회 중 5회, 판정은 매 회차 그 순간 장면과 일치했습니다. 상자 마커를 못 읽은 1회는 보조 판정이 상차를 잡았습니다. 카메라 확인은 팔 복귀 뒤 약 2~7초에 끝나므로, 결과(`completed`/`failed`)가 나오기 전에 상자를 옮기면 안 됩니다.

### 실패 뒤 홈 복귀 안전 조건

`MISSION_FAILED`의 홈 복귀(`/api/home/return`)는 제어판 armed가 켜져 있을 때만 요청합니다. 제어판의 홈 이동 자체는 armed를 보지 않으므로, armed가 꺼져 있으면(작업자 정지, 이상 감지) Host는 팔을 움직이지 않고 `PANEL_NOT_ARMED_FOR_HOME`, `MISSION_HOLD`로 보고합니다.

### AI 추론 연결 확인

추론은 제어판이 GPU AI 추론 서버로 UDP(`RPI1`)만 사용합니다(제어판 3.13.0부터). Host는 제어판 `/api/status`의 `gpu_link.infer`를 읽어 다음이 모두 맞을 때만 준비 상태입니다. Host는 추론 서버에 직접 PING하지 않습니다(공유 키를 제어판 한 곳에만 둠).

- `transport == "udp"`, `udp.ok is True`, 마지막 확인 시각 `udp.at`이 20초 이내(제어판은 약 5초마다, 느리면 최대 15초마다 확인).
- `ok`가 `null`(시작 중, GPU 서버가 이 PC를 아직 허용 전)이어도 준비 아님입니다. 제어판은 `ok: false`일 때만 평가 시작을 거절하므로 Host가 먼저 막습니다.
- 준비가 아니면 `executor_error`에 `AI_LINK_DOWN`·`AI_LINK_STALE`·`AI_LINK_NOT_UDP`·`AI_LINK_UNKNOWN`을 싣고, 작업 명령은 접수 전에 `EXECUTOR_ERROR`로 거절합니다(HOLD 없음).
- PING 성공은 서버가 살아 있다는 뜻이며 첫 추론의 모델 적재 지연까지 보장하지 않습니다.
