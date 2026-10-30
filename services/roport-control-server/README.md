# roport-control-server

RO:PORT · GATE CREW — 통합관제 및 로봇 통신

## 기술 스택

Python 3.12 / uv / Paho MQTT 2.1.0 / pytest / Ruff.
최종 구조는 PyQt UI ↔ 관제 실행부 직접 연결이며, 실행부만 DB에 접근합니다.
기존 FastAPI `/healthz` 뼈대는 런타임 전환 PR에서 제거할 예정입니다.

## 현재 범위

MQTT 5 작업 명령·이벤트 모듈과 모의 Pinky·OMX 왕복 예제를 제공합니다.
실행부/IPC·DB 연결·기본 PyQt·UDP 영상은 후속 PR입니다. DB 자체와 업무 CRUD는 만들지 않습니다.
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
uv run ruff check .
uv run pytest
uv build
```

API 실행: `uv run uvicorn roport_control_server.app:app --host 127.0.0.1 --port 8000`

`/healthz`는 프로세스 상태만 확인합니다. 로봇·DB·모델 준비 상태를 뜻하지 않습니다.

## MQTT 모듈과 팀원 이식

- `src/roport_control_server/communication/messages.py`: 공통 envelope, 토픽, 명령 검증 및 메모리 중복 기록.
- `src/roport_control_server/communication/mqtt.py`: Paho MQTT 5 연결, 재구독, 제한된 큐, 수동 transport ACK, TLS 설정.
- `src/roport_control_server/examples/mqtt_demo.py`: 관제 송신 및 가짜 Pinky·OMX 수신 예제. 실제 로봇 실행 코드가 아닙니다.

팀원은 **communication 폴더 전체**를 자신의 Python 패키지에 복사하고 `paho-mqtt==2.1.0`만 추가할 수 있습니다. 내부 상대 import만 사용하며 Qt/ROS/DB/FastAPI를 불러오지 않습니다. 송수신 사용 흐름은 예제의 `run_control`/`run_device`를 참고하고 `MockExecutor`는 실제 실행부로 교체합니다.

`MqttConfig(host, client_id, port=1883)`는 로컬 Mosquitto용입니다. EMQX Cloud에서는 같은 코드를 쓰고 `host`, `port=8883`, `tls=True`, `username`, `password`를 설정합니다. 자격증명은 환경변수/배포 secret에서 읽으세요. 사설 CA가 필요하면 `ca_file`을 지정합니다. 인증서·호스트명 검증을 끄는 옵션은 없으며, 암호 인증은 TLS를 요구합니다. **Cloud 실제 접속·ACL·WAN 장애는 아직 검증하지 않았습니다.**

현재 cmd/event만 구현합니다. telemetry·presence/LWT·durable application `/ack`는 후속 실행부 연동 범위입니다. 기체·작업대별 명시적인 토픽을 구독하며 OMX1·2는 `workcell/shelf-01` 하나를 사용합니다. 개별 `robot/omx1`, `robot/omx2` 명령은 허용하지 않습니다.

### 로컬 왕복 실행 (터미널 3개)

Mosquitto 2.x 실행 파일을 준비한 뒤 아래 순서대로 실행합니다. 별도 브로커를 loopback 전용 포트에 띄우며 기존 현장 브로커 설정은 바꾸지 않습니다.

```bash
# 1. 브로커
printf 'listener 18883 127.0.0.1\nallow_anonymous true\npersistence false\n' > /tmp/roport-test-mosquitto.conf
mosquitto -c /tmp/roport-test-mosquitto.conf

# 2. 모의 수신기 (이 패키지 디렉토리에서)
uv run python -m roport_control_server.examples.mqtt_demo pinky1 --port 18883

# 3. 관제 송신기
uv run python -m roport_control_server.examples.mqtt_demo control --device pinky1 --port 18883
```

OMX3 시험은 두 명령의 `pinky1`을 `omx3`로, OMX1·2 공동 작업대는 `shelf-01`로 바꿉니다. `accepted`와 `completed` 모의 이벤트를 확인하고 각 터미널에서 Ctrl+C로 종료합니다. 모든 예제는 `127.0.0.1`, site=`test`, 모의 허가만 사용합니다. Pinky의 `test-v1` 경로와 OMX `load` payload는 **시험용 부분 계약**이며 최종 현장 명령 계약이 아닙니다.

### 처리 순서와 한계

1. `start()`는 MQTT 연결·SUBACK까지 기다립니다. `ready`는 기체 준비나 작업 승인 상태를 의미하지 않습니다.
2. `receive()` → `decode()` → 수신자·권한·현재 작업 검증 → 실행부 처리 → `ack(delivery)` 순서로 소비합니다. 네트워크 callback에서는 큐에 넣기만 합니다. 잘못된 메시지는 거절/기록 후 ACK하며, 정상 메시지의 영속화가 필요하면 호출자가 commit을 먼저 완료해야 합니다.
3. `publish()` 성공은 브로커 PUBACK입니다. accepted/completed/DB 반영과 구분하세요. cmd/event는 QoS1·retain=false이고 cmd에는 MQTT5 expiry와 수신 측 expires_at 검사 둘 다 적용합니다. 컴퓨터 간 시각 동기화가 필요합니다.
4. 단절은 자동 재연결·재구독합니다. 이전 연결의 큐 항목/ACK는 거절합니다. 큐 초과는 연결을 닫고 오류로 알립니다. 호출자는 상태를 재조정한 후 새 transport를 만들어야 합니다. timeout·단절 직전 발행은 전달 여부가 불명확하며 Paho가 미확인 QoS 메시지를 다시 보낼 수 있으므로 동일 command_id로 대조합니다.
5. `CommandHistory`는 단일 소비자·메모리 전용이며 가득 차면 새 명령을 거절합니다. **프로세스 재시작 중복 방지·exactly-once·DB 복구를 보장하지 않습니다.** 실로봇 이식 전 실행 착수 기록의 영속화, 재시작 재조정, 자원 허가·도킹·모델/보정 검증 및 현장 정지 처리가 필요합니다. 이 PR은 모터를 구동하지 않습니다.

기본 시험 한도는 메시지 64 KiB·수신/송신 큐 128개이며 운영 부하 기준은 아직 확정하지 않았습니다. 모의 공동 작업대에서는 양팔 복귀가 모두 확인돼야 completed가 됩니다. 관제의 실제 출발 승인은 후속 구현입니다.

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
