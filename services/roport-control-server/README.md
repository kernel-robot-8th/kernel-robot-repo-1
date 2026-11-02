# roport-control-server

RO:PORT · GATE CREW — 통합관제 및 로봇 통신

## 기술 스택

Python 3.12 / uv / Paho MQTT 2.1.0 / pytest / Ruff.
통합관제 Tech Spec v4의 프로세스 구성은 PyQt UI ↔ FastAPI ↔ 운영 DB ↔ 관제 실행부(별도 프로세스)이며, API와 실행부는 운영 DB로만 연결합니다. 원격 서비스 WSS는 유지합니다.
현재 구현은 FastAPI `/healthz` 뼈대·PyQt 화면·MQTT/UDP 모듈뿐이며 운영 DB·관제 실행부·WSS 연결은 아직 없습니다.

## 구조 · FFT1-122

```
src/roport_control_server/
  domain/      __init__.py만: Workflow·Allocation·State/Recovery 규칙 자리 (골격, 미구현)
  executor/    __init__.py만: 관제 실행부 프로세스 자리 (골격, 미구현, FFT1-106)
  contracts/   snapshot.py: TaskRow·Snapshot 표시 계약과 검증 (표준 라이브러리만)
  adapters/
    mqtt/      messages.py·mqtt.py: MQTT 5 명령·이벤트 (폴더 단위 이식)
    udp/       udp_frame_protocol.py·udp_frame_sender.py·udp_frame_receiver.py: UDP 영상 wire v1
    api/       app.py: FastAPI /healthz
    ui/        window.py·__main__.py: PyQt 관제 화면
scripts/       mqtt_demo.py·udp_video_demo.py: 모의 예제 (wheel 미포함, MockExecutor 포함)
tests/         test_layers.py: 아래 import 규칙 검사
```

| 위치 | 허용 import |
| --- | --- |
| `contracts/` | 표준 라이브러리 |
| `adapters/mqtt/` | 표준 라이브러리, paho, 자기 폴더 |
| `adapters/udp/` | 표준 라이브러리, 자기 폴더 |
| `adapters/ui/` | 표준 라이브러리, PyQt6, `contracts` |
| `adapters/api/` | 표준 라이브러리, fastapi, `contracts` |
| `domain/` (골격) | 표준 라이브러리 |
| `executor/` (골격) | 표준 라이브러리, `domain`, `contracts`, `adapters`(ui·api 제외) |

패키지는 `scripts`·`tests`를 import하지 않고, DB 드라이버(psycopg·sqlite3 등)는 `adapters/db/` 밖에서 import하지 않습니다. `domain/`·`executor/`는 책임만 적은 골격이고 코드는 아직 없습니다. `adapters/db/`·`adapters/wss/`는 DB 종류와 WSS 메시지 계약이 확정된 뒤 만듭니다. 규칙이 없는 새 폴더는 `tests/test_layers.py`가 실패시키므로 규칙을 함께 추가합니다.

## 현재 범위

MQTT 5 작업 명령·이벤트 모듈과 모의 Pinky·OMX 왕복 예제, 독립 실행 가능한 기본 PyQt 관제 화면을 제공합니다.
UDP 영상 송수신 모듈(FFT1-105)도 포함합니다. 관제 실행부·운영 DB 연결은 후속 PR입니다. DB 자체와 업무 CRUD는 만들지 않습니다.
CI 성공은 실로봇·GPU·통합 검증 완료를 의미하지 않습니다.
배포 환경 미정으로 CD와 자동 배포는 설정하지 않았습니다.

## 기본 PyQt 통합관제 UI · FFT1-107
`src/roport_control_server/adapters/ui/window.py`는 실행부와 독립된 표시용 화면이며 표시 계약 `TaskRow`·`Snapshot`은 `contracts/snapshot.py`에 있습니다. 기본 실행은 **OFFLINE**, `--demo`는 명시적인 모의 데이터입니다. 작업 현황·오류·상태 갱신 시각과 Pinky/OMX 탭을 제공합니다. 작업 승인·출발·취소 버튼은 실행부 연동 전까지 비활성화합니다. UI가 MQTT/UDP/DB에 직접 접근하지 않습니다.

```bash
uv sync --locked --extra ui
uv run --extra ui python -m roport_control_server.adapters.ui          # 상태 없음
uv run --extra ui python -m roport_control_server.adapters.ui --demo   # 모의 데이터
```

PyQt6 6.12.0을 선택 의존성 `ui`로 고정했습니다([공식 설치 안내](https://www.riverbankcomputing.com/software/pyqt/download)). 팀원의 상세 위젯도 같은 PyQt6 환경을 사용해야 합니다. Linux 실제 창 실행에는 데스크톱과 Qt platform 라이브러리가 필요하며 headless 시험은 `QT_QPA_PLATFORM=offscreen`을 사용하며 Ubuntu의 `libegl1`이 필요합니다(CI에서 설치).

### 팀원 탭·후속 상태 입력 연결점

- `ControlWindow.set_detail_widget('Pinky' 또는 'OMX', widget)`: 부모가 없는 PyQt6 QWidget을 해당 탭에 넣습니다. 다른 탭과 선택 상태는 유지합니다. 상세 로봇 제어 코드는 각 팀원이 담당합니다.
- `set_source(session_id)`: 표시할 상태의 세션을 명시적으로 설정하고 이전 화면을 비웁니다. `None`은 미연결 표시입니다. 통신 연결을 생성하지 않습니다.
- `apply_snapshot(Snapshot(...))`: 전체 표시 상태를 교체합니다. 같은 세션의 증가한 revision만 받으며 중복·과거·다른 세션은 무시합니다. `TaskRow`의 진행률·문자열·목록 크기를 검증하며 최대 작업 100개/오류 20개를 표시합니다. 오류는 plain text로 표시합니다.
- 새 snapshot 없이 5초가 지나면 갱신 지연을 표시합니다. 이 값은 UI 시험 기준이며 로봇 timeout·준비 판정이 아닙니다. 모든 화면 메서드는 GUI 스레드에서 호출하며, 향후 통신 스레드는 Qt queued signal로 전달해야 합니다.

실행부·IPC·WSS·DB 연결, 실제 UI 재접속 복구, MQTT/UDP 동시 부하 및 실기 통합은 후속 범위입니다. 이번 PR은 화면 뼈대와 위젯/데이터 입력 지점만 제공합니다.

검증: `uv sync --locked --extra ui`, `uv run ruff check .`, `uv run ruff format --check .`, `env -u PYTHONPATH QT_QPA_PLATFORM=offscreen REQUIRE_UI_TESTS=1 uv run --extra ui pytest`, `uv build`. CI는 ui extra 설치와 UI 테스트 실행을 필수로 하며, 기본 의존성만 설치한 환경에서는 UI 테스트를 skip합니다. [Jira FFT1-107](https://fp-team1.atlassian.net/browse/FFT1-107).

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

3D 표시 자산은 [FFT1-109](https://fp-team1.atlassian.net/browse/FFT1-109)의 선행 패키징입니다. 화면 구현은 후속 PR이며 현재 기본 PyQt 진입점은 유지합니다.
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

- `src/roport_control_server/adapters/mqtt/messages.py`: 공통 envelope, 토픽, 명령 검증 및 메모리 중복 기록.
- `src/roport_control_server/adapters/mqtt/mqtt.py`: Paho MQTT 5 연결, 재구독, 제한된 큐, 수동 transport ACK, TLS 설정.
- `scripts/mqtt_demo.py`: 관제 송신 및 가짜 Pinky·OMX 수신 예제(`MockExecutor`·`make_command`, 시험도 같은 정의를 사용). 실제 로봇 실행 코드가 아니며 wheel에 포함되지 않습니다.

팀원은 **adapters/mqtt 폴더 전체**를 자신의 Python 패키지에 복사하고 `paho-mqtt==2.1.0`만 추가할 수 있습니다. 내부 상대 import만 사용하며 Qt/ROS/DB/FastAPI를 불러오지 않습니다. 송수신 사용 흐름은 예제의 `run_control`/`run_device`를 참고하고 `MockExecutor`는 실제 실행부로 교체합니다.

`MqttConfig(host, client_id, port=1883)`는 로컬 Mosquitto용입니다. EMQX Cloud에서는 같은 코드를 쓰고 `host`, `port=8883`, `tls=True`, `username`, `password`를 설정합니다. 자격증명은 환경변수/배포 secret에서 읽으세요. 사설 CA가 필요하면 `ca_file`을 지정합니다. 인증서·호스트명 검증을 끄는 옵션은 없으며, 암호 인증은 TLS를 요구합니다. **Cloud 실제 접속·ACL·WAN 장애는 아직 검증하지 않았습니다.**

현재 cmd/event만 구현합니다. telemetry·presence/LWT·durable application `/ack`는 후속 실행부 연동 범위입니다. 기체·작업대별 명시적인 토픽을 구독하며 OMX1·2는 `workcell/shelf-01` 하나를 사용합니다. 개별 `robot/omx1`, `robot/omx2` 명령은 허용하지 않습니다.

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
