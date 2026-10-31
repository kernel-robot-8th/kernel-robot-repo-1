# roport-control-server

RO:PORT · GATE CREW — 작업 배정·상태 관리·ROS 2 Bridge

## 기술 스택

Python 3.12 / FastAPI / Uvicorn / uv / pytest / Ruff. 작업 상태 저장은 PostgreSQL 기준이며 DB 연결·마이그레이션은 후속 구현입니다. ROS 2 Jazzy Bridge는 API와 별도 프로세스로 실행합니다.

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

## UDP 영상 모듈 · FFT1-105
`communication/udp_video.py`는 OMX 카메라 호스트 → AI 수신기로 인코딩된 JPEG/PNG 프레임을 직접 보냅니다. `FrameInfo`, `fragment_frame`, `FrameSender`, `FrameReceiver`는 표준 라이브러리만 사용하므로 이 파일 또는 communication 폴더를 팀원 패키지에 복사할 수 있습니다. 카메라 캡처·인코딩·AI 추론·관제 실행부·UI 영상 표시는 구현하지 않습니다. MQTT 작업 명령/중요 결과와 별도 경로입니다.

- 입력: frame bytes, source/camera/stream, session UUID, 증가하는 frame_id, captured_ns, observation_group, codec. 같은 물품 관측의 두 카메라는 observation_group을 공유할 수 있습니다. frame_id는 세션 내 스트림별 단조 증가이며 재사용하지 않습니다.
- 시험 프로토콜 v1: network byte order, 120 B 고정 헤더 + 최대 1080 B 조각. datagram 1200 B·프레임 1 MiB·독립 JPEG/PNG만 지원합니다. RTP/H.264 호환 프로토콜은 아닙니다. 실제 영상 포맷 검사는 디코더가 담당합니다.
- 수신기는 지정한 IPv4+송신 포트, source/camera/stream, session만 받습니다. 새 세션은 담당 코드에서 확인한 뒤 수신기를 새로 구성합니다. IP 허용 목록·CRC32는 암호학적 인증이 아닙니다. **모의 LAN용이며 현장 인증·키 설정·암호화·MTU는 별도 검증 대상**입니다.
- 스트림당 미완성 프레임 1개만 유지합니다. 최신 프레임이 도착하면 이전 미완성 프레임을 폐기하고 중복·역순 조각을 처리합니다. 기본 1.5초 deadline은 최초 조각 도착 기준이고 중복으로 연장하지 않습니다. 완료/만료한 frame_id는 재조립하지 않습니다. captured_ns는 추적용이며 호스트 간 절대 지연 검사는 별도 시각 동기화가 필요합니다.
- 최대 32개 허용 스트림, 기본 inflight 16개·예약 payload 8 MiB입니다. 조각 수·크기를 검사한 뒤 할당하고 완료 시 최대 1 MiB의 출력 사본 및 Python 객체 오버헤드가 추가됩니다. `stats`에 수신 datagram·폐기 원인·완료 프레임/바이트 수가 쌓이고 `last_reassembly_seconds`에 수신 호스트 기준 마지막 재조립 시간이 기록됩니다. 트래픽이 없어도 수신 루프에서 `expire()`를 주기적으로 호출하세요.

`FrameSender`는 카메라별 FPS와 IP/UDP 헤더를 포함한 비트율을 제한합니다. 기본 8 Mbps에서 최대 1 MiB 프레임의 전송 시간은 약 1.19초이므로 기본 수신 timeout은 여유를 둔 1.5초입니다. 재조립이 끝나면 즉시 반환하며, 1.5초는 미완성 프레임의 최대 대기 시간입니다. FPS는 상한이며 큰 프레임에서 설정 FPS를 보장하지 않습니다. 비트율을 낮추거나 timeout을 줄일 때는 `(프레임 바이트 + ceil(프레임 바이트 / 1080) × 148) × 8 / bitrate`에 스케줄링·네트워크 지연 여유를 더해 설정하세요. 재전송·FEC·혼잡 제어는 없으며 9대 카메라의 허용 비트율 합은 실제 Wi-Fi 측정 후 정합니다.

### 원본과 이식본 관리

- 원본은 이 저장소의 `services/roport-control-server/src/roport_control_server/communication/udp_video.py`이며 통합관제 담당자가 관리합니다. 영상 경로는 OMX → AI 직접 전송으로 유지합니다.
- OMX·AI 담당자는 같은 원본 리비전의 파일을 수정 없이 복사하고, 각 패키지 README에 원본 경로·전체 Git commit SHA·파일 SHA-256을 기록합니다. 캡처·추론 등 장비별 코드는 별도 호출부에 둡니다.
- 공통 변경은 원본 PR에서 검증한 뒤 OMX 송신부와 AI 수신부의 이식 PR을 연결합니다. 양쪽 파일 해시가 원본과 일치하고 송수신 검증을 통과한 뒤 함께 적용합니다. 사본에 독자적인 프로토콜 변경을 넣지 않습니다.
- 호환되지 않는 헤더/필드 의미 변경은 wire 버전을 올리고 송수신 양쪽을 갱신합니다. 현재 수신기는 `RPV1`만 허용하며 버전 불일치를 폐기합니다.

### 로컬 송수신 예제

두 터미널에서 같은 새 session UUID를 사용합니다. 아래 UUID는 1회 시험 예시이며 다시 실행할 때는 새 값을 쓰세요. 파일은 직접 준비한 JPEG/PNG입니다. mock AI는 수신된 바이트 수와 통계만 출력합니다.

```bash
# 터미널 1
uv run python -m roport_control_server.examples.udp_video_demo receive --session 673593ea-50ba-4c6d-ab47-8b8d209ee038
# 터미널 2
uv run python -m roport_control_server.examples.udp_video_demo send --session 673593ea-50ba-4c6d-ab47-8b8d209ee038 --file /absolute/path/frame.jpg --frames 10 --fps 10 --bitrate 8000000
```

예제는 loopback의 5301(수신)·5302(송신) 포트만 사용합니다. `--port`, `--source-port`, `--camera`, PNG용 `--codec png`를 지원합니다. 실제 적용 시 호출자가 socket과 허용 스트림/세션을 구성하고 장비별 카메라 ID를 합의해야 합니다.

검증: `uv sync --locked`, `uv run ruff check .`, `uv run ruff format --check .`, `env -u PYTHONPATH uv run pytest`, `uv build`. 단위·loopback·별도 프로세스 예제 시험이며 실제 카메라/AI·현장 처리량 측정은 포함하지 않습니다. [Jira FFT1-105](https://fp-team1.atlassian.net/browse/FFT1-105).

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
