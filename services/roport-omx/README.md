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

## OMX 제어판 연동 기준

Host 프로그램이 함께 동작하는 제어판 버전, 사용하는 제어판 API, 현장 설정 파일은 [docs/control-panel.md](docs/control-panel.md)에 기록합니다.

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
- 제어판 [HOLD 해제] 버튼(같은 PC, 같은 사용자 파일 통로, 네트워크 아님): Host는 journal 옆에 `<journal>.host-state.json`(boot_id, 진행 중 작업, HOLD와 원인 `error_code`·`detail`)을 쓰고, 제어판이 `<journal>.hold-request.json`(`request_id`, 화면에 보인 `hold`, 확인 체크 `arms_safe`·`package_checked`·`pinky_checked`, `requested_at`)을 쓰면 한 번 읽고 지운 뒤 `<journal>.hold-result.json`에 결과를 씁니다. 같은 HOLD·체크 3개·진행 중 작업 없음·실행부 유휴·양팔 홈과 그리퍼 놓음(실측)이 모두 맞아야 해제하며, 아니면 사유(`HOLD_CHANGED`, `CHECKS_MISSING`, `TASK_ACTIVE`, `EXECUTOR_BUSY`, `ARMS_NOT_HOME`, 60초 지난 요청 `STALE_REQUEST`)와 함께 HOLD를 유지합니다. 해제해도 armed는 켜지지 않습니다. `--clear-hold` 재시작도 그대로 쓸 수 있습니다.
- 호스트 메트릭(표시 전용): `--host-metrics-period-s 5`처럼 주기를 줄 때만 `roport/workcell/shelf-01/host`에 v2 `host_metrics`(QoS0, retain=false, boot_version=boot_id)를 발행합니다. 생략하면 꺼지며 기본값은 없습니다. 작업·ACK·HOLD·정지 판단에 쓰지 않고 발행 실패는 로그만 남깁니다. 실장비·실브로커에서는 아직 검증하지 않았습니다.
- 실물 실행: `--executor panel --routes <경로 파일>`은 같은 PC의 OMX 제어판(v3.14.0 이상) API로 평가 1회와 이어지는 선반 정렬을 시작합니다(`shelf_align: true`). 칸마다 모델이 다르므로 `(operation, slot_id, manipulation_mode)`별 모델·작업 문장·시간을 경로 파일에 둡니다(`config/cooperative_routes.example.json`, `manipulation_mode`는 생략하면 그 칸의 모드). 경로에 `markers`(그 경로가 받을 상자 마커 목록)를 두면 다른 상자는 `MODEL_NOT_READY`로 거절합니다. 2026-11-18 현장 경로는 양팔 상자 16·17·20과 OMX3가 다루는 15·16·17·21·22를 합친 15·16·17·20·21·22입니다. 경로마다 제어판 출력 필터(`output_filter`: 약·중·강, 생략하면 끔)와 `filter_gripper`를 지정하면 관제로 시작한 평가에도 그대로 적용합니다. 경로가 없는 칸, 학습 목록에 없는 모델, 제어판 armed 꺼짐은 동작 전에 거절합니다. 완료는 제어판 `last_result`의 정상 종료·양팔 복귀·그리퍼 놓음(52+5% = 54.6 이하)으로 판정합니다. 제어판 3.12부터 Host는 `end_on_return`을 보내, 팔이 처음 자세에서 벗어났다가 돌아오고(모든 관절 8 이내) 그리퍼가 54.6 이하로 0.3초 유지되면 `episode_s` 전이라도 평가를 끝냅니다(정책의 재시도 방지). 조기 종료 신호(`last_result.last_episode.returned`)만으로 안전 복귀를 확정하지 않습니다. 완료·실패 구분은 아래 "선반 적재 미션 판정"을 따릅니다. 제어판 3.9부터 평가 뒤 선반 정렬이 이어지면 `final`이 될 때까지 기다리고, 정렬 성공·정렬 뒤 쉬는 자세 확인(`align.rest`)·최종 `/api/arms` 홈/그리퍼 실측을 함께 판정합니다(`evidence: "shelf_align"`). 사람 조치를 기다리는 `hold`(비상 정지, 내려놓기 경로 없음)는 기다리지 않고 바로 실패·HOLD로 보고합니다. 제어판에서 사람이 누른 중지·즉시 중단은 정지가 끝난 뒤 `PANEL_STOPPED`·`PANEL_ABORTED`(정렬 중이면 `PANEL_ALIGN_STOPPED`)로 실패 보고하되 HOLD하지 않습니다. 정지하면 제어판 armed가 꺼지므로 다음 시작은 사람이 armed를 다시 켜야 합니다. 비상 정지·강제 종료·내려놓기 실패·오류·미복귀는 HOLD합니다. 상태 조회(`status.requested`)의 양팔 상태는 제어판 3.11의 `GET /api/arms?model=…`(실행 중이 아닐 때 모터 위치를 읽음)에서 가져오며, 경로표의 모든 모델 홈 자세 안이어야 `returned`입니다. 그리퍼는 54.6 이하면 놓음, 넘으면 `grasping`으로 봅니다(2026-11-10 시험 합의: 빈 채로 닫히면 48~51, 상자에 막히면 58~60, 열림 61~67. 상자 모서리를 50 근처에서 쥘 수도 있어 놓음 여부는 팔 복귀와 함께 판정). 제어판 3.11부터 armed를 켤 때 `~/.config/roport/omx-host.json` 설정(예: `config/omx-host.example.json`, `link_config`는 `execution_mode: hardware`여야 함)으로 이 Host를 자동 시작하고, armed를 꺼도 Host는 남아 보고·상태 조회에 답합니다. Host는 제어판 **3.14.0 이상**이고 AI 추론 UDP 연결이 살아 있을 때만 준비 상태로 보고합니다(아래 "AI 추론 연결 확인"). **팔이 움직이므로 사람이 옆에서 armed를 켠 상태로만 시험합니다.**
- 준비 조회·모델 검사 이후, 접수 journal 저장 이후와 `/api/eval/start` POST 직전에 각각 `expires_at`을 재검사합니다. 만료되면 새 평가를 시작하지 않습니다. HTTP 시작 응답이 끊기면 결과 불명·HOLD이므로 자동 재시작하지 않습니다.
- 검사: `bash ci/test.sh` (`ci/test_cooperative.py`·`ci/test_host_metrics.py`·`ci/test_panel_executor.py`; 브로커·제어판·장비 불필요). 실제 브로커 왕복은 로컬 Mosquitto에서만 확인했고, 패널 연동·실물 팔은 미검증입니다.

### 명령의 상자와 칸 (2026-11-18 합의)

- `package_id`는 상자 윗면 ArUco 마커 번호의 문자열입니다(예: `"17"`, OMX3 `omx3/target.py`와 같은 규칙: ASCII 숫자, 앞자리 0 없음). 포장물품 마커 15·16·17·19·20·21·22만 받고(18은 Pinky 쟁반 마커, `docs/aruco/markers.yaml` v1.6), 그 밖은 움직이지 않고 `INVALID_TARGET`으로 거절합니다. 이 번호가 작업 문장의 `{box}`와 제어판 `target_marker`가 됩니다. `--box-marker`는 더 쓰지 않습니다(예전 시작 명령 호환으로만 남김).
- 칸이 팔을 정합니다(선반 2×3, 1 = 아래층, 2 = 위층, A·B·C = 팔 쪽에서 볼 때 왼쪽부터).

| 칸 | `manipulation_mode` | `assigned_arm_id` | `robot_ids` |
| --- | --- | --- | --- |
| 1B·2B | `COOPERATIVE` | `null` | `["omx1", "omx2"]` |
| 1A·2A | `DEDICATED` | `"omx1"` | `["omx1"]` |
| 1C·2C | `DEDICATED` | `"omx2"` | `["omx2"]` |

- 명령의 팔 배정이 표와 다르면 `SLOT_ARM_MISMATCH`, `operation`·`workcell_id`가 맞지 않으면 `WORKCELL_MISMATCH`로 거절합니다. 그 칸·모드의 경로(모델)가 없으면 `UNSUPPORTED_SLOT`입니다(한 팔 모델은 데이터 수집 예정).
- `completed`의 `departure_ready: true`는 OMX3 `load.completed`와 같은 뜻(Pinky가 바로 출발해도 됨)입니다. 하차는 팔이 홈으로 돌아오고 그리퍼를 놓은 뒤에만 보냅니다(팔이 Pinky 위에 있으면 출발하면 안 됨). 적재는 `--report-placement`면 상자가 칸에 놓인 순간 보냅니다(아래 두 단계 보고).

### 선반 적재 미션 판정 (2026-11-16 합의)

정렬 요청과 이 적재 판정은 입고(`shelf_inbound`)에만 적용합니다. 하차는 아래 "하차 미션 판정"을 따릅니다.

선반 적재 미션은 적재 뒤 상자 정렬까지 포함합니다. Host는 시작 요청에 `shelf_align: true`를 보내고, 평가·정렬 결과(`last_result.align`)와 양팔 실측으로 판정해 이벤트 payload의 `mission_result`로 보고합니다.

| `mission_result` | 조건 | 이벤트 | HOLD |
| --- | --- | --- | --- |
| `MISSION_COMPLETE` | 정렬 성공(명령한 칸), 양팔 홈·그리퍼 놓음 | `completed`, `evidence: "shelf_align"` | 없음 |
| `MISSION_COMPLETE_NOT_ALIGNED` | 상자가 선반 위(`on_shelf: true`), 정렬은 시간 초과·실패·건너뜀, 양팔 홈·그리퍼 놓음 | `completed`, `evidence: "shelf_load_not_aligned"`, `align_error` | 없음 |
| `MISSION_FAILED` | 평가 오류, 상자가 선반 위에 없음(`PANEL_BOX_NOT_ON_SHELF`), 다른 칸 정렬(`PANEL_ALIGN_WRONG_SLOT`), 팔 미복귀·그리퍼 쥠(`ARM_NOT_RETURNED`), 작업자 중지·즉시 중단 | `failed` | 없음: 제어판 `/api/home/return`으로 양팔을 평가 시작 자세로 보낸 뒤 `/api/arms`로 홈·그리퍼 놓음을 확인하고 보고 |
| `MISSION_HOLD` | 선반 판독 불가(`on_shelf: null` → `PANEL_SHELF_UNREADABLE`, 정렬·판독 모두 없음 → `PANEL_ALIGN_SKIPPED`·`PANEL_ALIGN_NOT_RUN`), 비상 정지·강제 종료·내려놓기 실패, 결과 불명, 홈 복귀 거절·시간 초과·복귀 후에도 미복귀 | `failed` | 있음: 현장 확인 후 `--clear-hold` |

- 양팔 홈 판정: 정렬이 돌았으면 정렬 뒤 쉬는 자세 확인(`align.rest`), 돌지 않았으면 평가 결과의 홈 판정을 쓰고, 완료·실패 보고 전에 `/api/arms`로 최종 모터 위치·그리퍼를 다시 읽습니다. 이미 홈이면 이동 요청을 보내지 않습니다.
- 홈 복귀는 그리퍼를 열기 때문에, 쥐고 있던 상자는 놓습니다. 비상 정지로 멈춘 팔은 자동으로 움직이지 않습니다.
- 제어판 3.17.0부터 Host는 시작 요청에 명령의 칸(`"slot": "2B"`)과 `pinky_id`를 실어, 제어판이 그 칸에서 판독·정렬하고 그 Pinky의 쟁반 기준을 쓰게 합니다. 그보다 오래된 제어판은 PC 설정 칸(2B)에서만 정렬하므로 다른 칸 미션은 `PANEL_ALIGN_WRONG_SLOT`으로 실패합니다.
- 관제(`omx_trial`)는 `arm_return_only`(모의 실행부)·`shelf_placed`·`shelf_align`·`shelf_load_not_aligned`만 완료로 받고, Not Aligned 사유를 이송 기록의 `align_error`에 남깁니다. 관제는 현재 모든 `failed`를 HOLD로 처리합니다(`MISSION_FAILED` 후 관제 흐름은 관제 담당과 합의 필요).

#### 두 단계 보고: 놓였음 → 정렬 (2026-11-17 합의, `--report-placement`)

상자가 명령한 칸에 놓이면 바로 Mission Complete를 보내 Pinky가 떠날 수 있게 하고, 정렬은 후속 작업으로 따로 보고합니다. 기본은 꺼져 있으며(위의 한 번 보고), 관제가 `shelf_placed`를 받도록 바뀐 뒤 `--report-placement`로 켭니다.

1. 제어판(3.20.0+)이 `last_result.placement.on_shelf: true`(명령한 칸)를 남기고, 그 순간 **두 손이 상자에서 떨어졌으며**(넘겨받기 판정이면 두 손끝이 상자 앞면·선반 앞 경계보다 3cm 뒤 `tips_clear`, 아니면 평가가 복귀로 끝난 때 양팔 홈·그리퍼 54.6 이하 `eval_end`) **카메라 촬영 원시각이 3초 이내**(`observed_ts`, `observed_clock: "capture"`)일 때만, 정렬이 끝나기 전에 `completed`, `mission_result: "MISSION_COMPLETE"`, `evidence: "shelf_placed"`, `placement_confirmed: true`, `grasp_released: true`, `departure_ready: true`, `align_state: "PENDING"`, `placement`(칸·상자·촬영 시각·판정 근거)를 보냅니다. 근거가 하나라도 없으면 조기 보고 없이 정렬 뒤 한 번 보고합니다(관제 합의 2026-11-18). 제어판이 촬영 시각을 몰라 제어 주기 시각을 넣은 경우(`observed_clock: "control_step"`)나 시각 출처를 밝히지 않은 경우도 근거로 쓰지 않습니다(2026-11-21 리뷰). 지금은 넘겨받기 판정(`source: "eval_judge"`)만 `observed_clock`을 남기므로, 정렬 실행기가 읽은 놓였음은 조기 보고하지 않습니다. 관제의 촬영 시각·`ALIGN_HOLD` 검증 전에는 현장에서 `--report-placement`를 켜지 않습니다. 이때 팔은 아직 정렬 중이라 `arms`는 홈이 아닐 수 있습니다.
2. 정렬이 끝날 때까지 작업은 진행 중으로 남아, 새 명령은 `HOST_BUSY`로 거절합니다. 관제는 다음 선반 명령 전에 아래 정렬 결과(또는 `status.completed`의 `active: null`)를 기다려야 합니다.
3. 정렬 결과는 `progress` 이벤트(지금 관제는 `completed` 뒤의 `progress`를 저장하고 ack만 함)로 보냅니다.

| `phase`·`align_result` | 조건 | HOLD |
| --- | --- | --- |
| `ALIGNED` | 정렬 성공, 양팔 홈·그리퍼 놓음(`mission_result: "MISSION_COMPLETE"`) | 없음 |
| `NOT_ALIGNED` | 정렬 시간 초과·실패, 양팔 홈(`MISSION_COMPLETE_NOT_ALIGNED`, `align_error`) | 없음 |
| `ALIGN_FAILED` | 정렬 중 작업자 중지, 또는 팔 미복귀를 홈 복귀로 해결(`error_code`) | 없음 |
| `ALIGN_HOLD` | 비상 정지·내려놓기 실패·판독 불가 등 사람 확인 필요(`error_code`) | 있음 |

- 놓였음을 보기 전에 최종 결과(정렬 끝)가 먼저 나오면 위의 한 번 보고로 끝납니다. 다른 칸에서 읽혔거나 `on_shelf`가 `true`가 아니면 놓였음으로 보지 않고 최종 결과로 판정합니다.
- `--place-shadow`(제어판 3.18.0+): 넘겨받기 없이 놓음 판정만 돌려 기록합니다(실물 그림자 시험). `--end-on-place`와 같이 주면 넘겨받기가 우선입니다.
- `--end-on-place`(제어판 3.18.0+): 상자가 칸에 놓이고 두 손이 상자에서 떨어진 순간 평가를 끝내고, 정렬기가 그 자리에서 팔을 이어받습니다(넘겨받기). 제어판 3.18.0 전에는 보내지 않습니다.
  - 정렬기가 제때 이어받지 못해(`handover.state` `failed`·`stopped`) 제어판이 팔을 홈으로 보냈다면, 판정 순간 상자가 칸에 있었고(`placement.on_shelf: true`) 홈 복귀가 성공(`fallback_ok: true`)했을 때 `MISSION_COMPLETE_NOT_ALIGNED`(`align_error: PANEL_HANDOVER_FAILED`·`PANEL_HANDOVER_STOPPED`)입니다. 홈 복귀가 실패했거나 놓였음을 확인하지 못했으면 HOLD입니다.
  - 모터 설정이 달라 이어받지 않으면(`regs_mismatch`) 제어판이 토크를 켠 채 멈추고(stage `hold`) Host도 HOLD입니다.

### 하차 미션 판정 (2026-11-17 합의)

하차는 선반의 상품을 관제 명령으로 대기 중인 Pinky 쟁반에 상차하는 작업입니다(`shelf_outbound.requested`, 예: 2b 칸 모델 `outbound_act_re1112x37_60k_1113`). 하차 경로가 있으면 제어판 3.20.0 이상이 필요합니다(작업 직전 쟁반 사진). Host는 시작 요청에 `pinky_check: true`, `shelf_align: false`, `draw_target: false`, 명령의 `slot`(예: `"2B"`)·`pinky_id`를 보내고, 평가 뒤 제어판이 카메라만으로 확인한 결과(`last_result.pinky`)로 판정합니다. 시간 기준은 쓰지 않습니다.

- 칸 비었음(`slot_empty`): 선반 정면 카메라(`front_2`)로 명령한 칸에 상자가 남았는지 읽습니다(빈 선반 기준 사진과 비교). 결과의 `slot`이 명령한 칸이고 `slot_empty`가 `true`/`false`로 있을 때만 씁니다. 선반 전체 값(`on_shelf`)으로 대신하지 않습니다(없으면 `PANEL_SLOT_UNREADABLE`, 2026-11-21 리뷰).
- 상차됨(`on_pinky`): 위 카메라(`front`)로 다음 중 하나면 `true`입니다.
  1. 상자 윗면 마커(15~22)가 쟁반 영역 안에서 읽힘(`box_on_tray`, 기본)
  2. 쟁반이 그 Pinky의 빈 쟁반 기준 사진과 달라짐(`tray_changed` ≥ 0.25, 보조). 상자가 기울거나 과노출로 마커가 안 읽힐 때를 대비합니다.
- **쟁반에는 마커를 붙이지 않습니다.** 하차 정책은 마커 없는 쟁반으로 학습되어, 쟁반에 마커가 있으면 동작을 시작하지 않습니다.
- 빈 쟁반 기준 사진은 Pinky마다 한 번 찍습니다(정렬 실행기 `--step tray_check --pinky N --tray-ref-reset`). 기준 사진이 없으면 `on_pinky: null`(HOLD)입니다.
- 확인은 정렬 실행기 `--step pinky_read`가 하며 로봇을 움직이지 않습니다. 제어판은 비상 정지·강제 종료가 아니면 평가가 어떻게 끝났든 확인합니다.
- 제어판 3.20.0부터 하차 직전에 그 자리 쟁반을 찍어(작업 직전 사진) 비어 있는지 먼저 확인하고, 작업 뒤에는 **그 미션의 직전 사진하고만** 비교합니다(다른 기체·공통 사진은 쓰지 않음, 관제 합의). 직전 확인에서 쟁반이 비어 있지 않거나 읽지 못하면 제어판이 평가를 시작하지 않고, Host는 팔이 움직이지 않았으므로 HOLD 없이 `MISSION_FAILED`(`PANEL_TRAY_NOT_EMPTY`·`PANEL_TRAY_CHECK_FAILED`)로 보고합니다. 작업 뒤 판정은 `tray_reference_state: "ok"`이고 `pre_mission_match: true`일 때만 믿습니다. 값이 다르거나 **없으면** `PANEL_PINKY_NO_REFERENCE`로 HOLD합니다(2026-11-21 리뷰: 직전 사진 확인 없이 `tray_changed`만으로 Pinky가 출발할 수 있었음).
- 하차 평가에서는 위 카메라 관측에 목표 원을 그리지 않습니다(`draw_target: false`, 제어판 3.16.1+). 목표 원 없이 학습된 모델이라, 그리면 동작이 크게 느려집니다.

| `mission_result` | 조건 | 비고 |
| --- | --- | --- |
| `MISSION_COMPLETE` | 명령한 칸의 `slot_empty: true`, `on_pinky: true`(작업 직전 사진 기준 확인), 양팔 홈·그리퍼 놓음(확인 뒤 `/api/arms` 실측) | `evidence: "pinky_load"` |
| `MISSION_FAILED` | 칸에 상자가 남음(`PANEL_BOX_STILL_ON_SHELF`) 또는 칸은 비었는데 쟁반이 빔(`PANEL_BOX_DROPPED`) | 양팔 홈 복귀 후 HOLD 없음 |
| `MISSION_HOLD` | 칸 판독 불가·다른 칸 결과(`PANEL_SLOT_UNREADABLE`), 직전 사진 확인 없음(`PANEL_PINKY_NO_REFERENCE`), 쟁반 판독 불가(`PANEL_PINKY_UNREADABLE`), 확인 없음(`PANEL_PINKY_CHECK_NOT_RUN`), 칸과 쟁반 둘 다 상자(`PANEL_PINKY_CHECK_CONFLICT`), 비상 정지 | 현장 확인 후 HOLD 해제 |

현장 시험(2026-11-17, 제어판 3.17.0, 비동기 100 / 0.2 / latest_only, 필터 중): 정책 성공 6회 중 5회, 판정은 매 회차 그 순간 장면과 일치했습니다. 상자 마커를 못 읽은 1회는 보조 판정이 상차를 잡았습니다. 카메라 확인은 팔 복귀 뒤 약 2~7초에 끝나므로, 결과(`completed`/`failed`)가 나오기 전에 상자를 옮기면 안 됩니다.

### 실패 뒤 홈 복귀 안전 조건

`MISSION_FAILED`의 홈 복귀(`/api/home/return`)는 제어판 armed가 켜져 있을 때만 요청합니다. 제어판의 홈 이동 자체는 armed를 보지 않으므로, armed가 꺼져 있으면(작업자 정지, 이상 감지) Host는 팔을 움직이지 않고 `PANEL_NOT_ARMED_FOR_HOME`, `MISSION_HOLD`로 보고합니다.

### AI 추론 연결 확인

추론은 제어판이 GPU AI 추론 서버로 UDP(`RPI1`)만 사용합니다(제어판 3.13.0부터). Host는 제어판 `/api/status`의 `gpu_link.infer`를 읽어 다음이 모두 맞을 때만 준비 상태입니다. Host는 추론 서버에 직접 PING하지 않습니다(공유 키를 제어판 한 곳에만 둠).

- `transport == "udp"`, `udp.ok is True`, 마지막 확인 시각 `udp.at`이 20초 이내(제어판은 약 5초마다, 느리면 최대 15초마다 확인).
- `ok`가 `null`(시작 중, GPU 서버가 이 PC를 아직 허용 전)이어도 준비 아님입니다. 제어판은 `ok: false`일 때만 평가 시작을 거절하므로 Host가 먼저 막습니다.
- 준비가 아니면 `executor_error`에 `AI_LINK_DOWN`·`AI_LINK_STALE`·`AI_LINK_NOT_UDP`·`AI_LINK_UNKNOWN`을 싣고, 작업 명령은 접수 전에 `EXECUTOR_ERROR`로 거절합니다(HOLD 없음).
- PING 성공은 서버가 살아 있다는 뜻이며 첫 추론의 모델 적재 지연까지 보장하지 않습니다.

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

관측을 묶는 함수(`pack_observations`)를 `infer_link`에 두는 일은 OpenCV 의존성 때문에 후속 작업으로 남깁니다.
