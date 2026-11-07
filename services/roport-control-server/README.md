# roport-control-server

RO:PORT · GATE CREW — 통합관제 및 로봇 통신

## 기술 스택

Python 3.12 / FastAPI / uv / Three.js / Paho MQTT / pytest / Playwright / Ruff. 정확한 버전은 manifest와 lockfile을 따릅니다.
2026-11-03 팀 합의: **브라우저 웹 UI ↔ FastAPI ↔ 운영 DB ↔ 관제 실행부(별도 프로세스 1개)**. API와 실행부의 목표 연결은 운영 DB이며, 원격 서비스 WSS는 실행부가 담당합니다. AI 노트북에 서버를 배치하고 다른 PC에서 접속합니다. AI 추론은 API와 별도 프로세스로 운영합니다.
현재 구현은 **웹 UI·REST 조회·WebSocket 모의/OFFLINE 상태 스트림**과 독립 MQTT/UDP 모듈입니다. 운영 DB·WSS·카메라 웹 중계는 후속입니다. DB 구현은 보류하며, Pinky 시험은 **원본 웹 UI ↔ FastAPI ↔ Unix socket ↔ 시험 실행부 1개 ↔ MQTT**로 연결합니다. 시험 이력은 파일 journal에 저장합니다. 브라우저는 DB/MQTT/UDP에 직접 연결하지 않습니다.

## 구조 · FFT1-122

```
src/roport_control_server/
  domain/      __init__.py만: Workflow·Allocation·State/Recovery 규칙 자리 (골격, 미구현)
  executor/    pinky_trial.py·trial_socket.py: CLI 또는 웹 IPC로 조작하는 Pinky 시험 (운영 실행부는 후속, FFT1-106)
  contracts/   snapshot.py: TaskRow·Snapshot 표시 계약과 검증 (표준 라이브러리만)
  adapters/
    mqtt/      MQTT 5 명령·이벤트·trial-v1 스키마/수신기/저장 ACK (이식 키트 제공)
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

패키지는 `scripts`·`tests`를 import하지 않고, DB 드라이버(psycopg·sqlite3 등)는 `adapters/db/` 밖에서 import하지 않습니다. `domain/`은 골격이며 `executor/pinky_trial.py`는 웹 API와 IPC로 연결되는 DB 비의존 시험 실행부입니다. `adapters/db/`·`adapters/wss/`는 DB 종류와 WSS 메시지 계약이 확정된 뒤 만듭니다. 규칙이 없는 새 폴더는 `tests/test_layers.py`가 실패시키므로 규칙을 함께 추가합니다.

## 현재 범위

MQTT 5 작업 명령·이벤트 모듈과 모의 Pinky·OMX 왕복 예제, FastAPI에서 제공하는 3D 웹 관제 화면을 제공합니다.
UDP 영상 송수신 모듈(FFT1-105)도 포함합니다. 웹에 연결된 운영 실행부·운영 DB 연결은 후속 PR입니다. DB 자체와 업무 CRUD는 만들지 않습니다.
CI 성공은 실로봇·GPU·통합 검증 완료를 의미하지 않습니다.
배포 환경 미정으로 CD와 자동 배포는 설정하지 않았습니다.

## Pinky MQTT 주행 시험 · FFT1-56

1대/2대 시나리오를 독립 CLI로 실행합니다. `route → 더미 상하차 → N12 mode_switch → navigate`를 완료 응답으로 진행하며, 파일 이력·중복 방지·저장 ACK·오류 시 HOLD를 포함합니다. 기존 CLI와 원본 웹 UI에서 같은 실행부를 사용하며 운영 DB는 연결하지 않습니다. 웹 연결·조작 키·시험 범위는 [가이드 7.4절](docs/pinky-trial.md)을 참고하세요.

**팀원 재현 순서와 메시지/이식 방법은 [주행 시험 가이드](docs/pinky-trial.md)를 따르세요.** `tools/prepare_pinky_trial.py`로 네 설정을 한 번 생성하고 같은 `trial.json`을 양쪽에 배포합니다. `tools/build_pinky_mqtt_kit.py`는 Paho만 필요한 이식 ZIP·JSON Schema·19개 메시지 예제를 생성합니다.

실제 ROS 드라이버와 독립 정지 감시는 [Pinky PR #35](https://github.com/kernel-robot-8th/kernel-robot-repo-1/pull/35)가 담당합니다. 가상 시험은 이 관제 브랜치만으로 재현할 수 있습니다. 실물은 해당 핑키 코드와 새 이식 키트를 함께 사용하고 현장 방향·골·정지 검증을 완료해야 합니다. 생성한 hardware 설정은 기본 `field_verified=false`라서 구동을 허용하지 않습니다.

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
- 서버가 제공한 목록으로 화면을 만들지만 현재 목록은 6대 예시입니다. 로봇 등록/제외와 실제 장비 목록 연동은 후속입니다.
- Pinky 탭 > Pinky Settings(FFT1-124, 시안 v0.2): 등록(이름·IPv4·역할·ROS_DOMAIN_ID 0~101·IR 기준값)과 작업(연결 확인·도메인 읽기/적용·업데이트/릴리즈 설치·빌드) 화면입니다. `contracts/pinky_settings.py`가 입력 형식을 정합니다.
  OFFLINE에서는 `POST /api/pinky/settings/robots`·`/jobs`가 형식만 검사하고 `503 executor_not_connected`를 돌려줍니다(저장·실행 없음). DEMO는 예시 3대를 브라우저 안에서만 바꿉니다.
  실제 설치·도메인 적용은 실행부·운영 DB 연결 뒤에 붙이며, 그 전까지는 Pinky 테스트 콘솔 `mode:=fleet`에서 시험합니다.
- 기존 회백색 3열 구성, 3D/평면·회전·선택 장비 확대, 관절 모델과 작업·오류 표시를 제공합니다. 실기 URDF 보정과 양팔 협업 동기화는 검증하지 않았습니다. OFFLINE/DEMO의 승인·출발·취소는 비활성화합니다. TRIAL에서는 시험 실행부가 허용하는 출발·더미 승인·물품 요청·보류만 조작합니다.

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
