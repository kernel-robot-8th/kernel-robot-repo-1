# roport-omx

RO:PORT · GATE CREW — OMX 제어·협동 동작·시연 수집·학습

## 기술 스택

Python 3.12 코어 패키지 / uv / Ruff. 학습·정책 실행은 LeRobot·PyTorch를 사용하되 OMX 드라이버·CUDA 호환성을 확인한 별도 환경에서 버전을 고정합니다.

## 현재 범위

개발 시작용 구조와 CI를 제공합니다. 로봇 제어·업무 기능·모델 추론은 아직 구현하지 않았습니다.
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

OMX1·2 COOPERATIVE 작업 명령(`roport-omx-task-v1`, `roport/v1/{site}/workcell/shelf-01/{cmd,event,ack}`)을 받아 실행부로 넘기고 event를 회신합니다. MQTT 송수신은 관제 서버 `adapters/mqtt`를 `src/roport_omx/mqtt_link/`로 복사해 사용합니다(원본 커밋은 각 파일 첫 줄). 현재 실행부는 **모의 구현만** 있으며 패널·모터를 호출하지 않습니다.

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
- 검사: `bash ci/test.sh` (`ci/test_cooperative.py`·`ci/test_host_metrics.py`, 브로커·장비 불필요). 실제 브로커 왕복은 로컬 Mosquitto에서만 확인했고, 패널 연동·실물 팔은 미검증입니다.

## AI 추론 UDP 클라이언트 (시험 단계, FFT1-154)

`src/roport_omx/infer_link/`는 Host PC가 GPU AI 추론 서버(`services/roport-ai-server`의 `omx_udp`)와 UDP 규약 `RPI1`로 통신하는 클라이언트입니다. 규약 파일 `wire.py`는 AI 서버 쪽 원본을 그대로 복사하며 두 곳을 함께 갱신합니다.

통신 구조: 관제 서버 ↔ Host PC는 MQTT, Host PC ↔ AI 추론 서버는 UDP, Host PC ↔ OMX 팔은 USB 시리얼(Dynamixel Protocol 2.0)입니다. Host PC 안에서 관제 Host(`cooperative_host.py`)는 로봇 루프를 가진 OMX 제어판을 로컬 HTTP로 호출하고, 제어판이 이 클라이언트로 추론을 요청합니다. 현장에서는 제어판(3.13.0 이상)이 같은 코드의 복사본을 사용하며 추론 통신은 UDP만 씁니다.

- 관측 한 묶음(관절 값 + 카메라 JPEG + 작업 정보)을 한 메시지로 보내고, 모든 조각이 모여 CRC·HMAC을 통과한 메시지만 씁니다.
- `infer()`는 요청마다 새 `observation_seq`와 `schema_version·boot_id·inference_id·captured_at·deadline`, 실행 중인 MQTT 명령의 `task_id·command_id·execution_id`를 붙입니다. 응답이 같은 시도·모델을 되돌려주지 않으면 `RESULT_MISMATCH`·`MODEL_MISMATCH`로 거절해 모터 목표로 쓰지 않습니다.
- 응답이 시간 안에 없으면 `InferenceTimeout`이며 스스로 재시도하지 않습니다. 호출자는 최신 관측으로 다시 요청하고 그동안 팔을 안전하게 유지합니다.
- 공유 키 파일(hex, `chmod 600`)은 저장소에 올리지 않습니다.
- 검사: `ci/test_infer_link.py`(loopback 가짜 서버). 제어판 연결, 실제 망 손실·지연, GPU 추론, 실물 팔은 이 검사에 포함하지 않습니다. 규약은 팀 AI UDP 설계안(Confluence MFS 10125435)의 원칙을 따르며 팀 합의 전 초안입니다.
