# roport-control-server

RO:PORT · GATE CREW — 작업 배정·상태 관리·ROS 2 Bridge

## 기술 스택

Python 3.12 / FastAPI / Uvicorn / uv / pytest / Ruff. 작업 상태 저장은 PostgreSQL 기준이며 DB 연결·마이그레이션은 후속 구현입니다. ROS 2 Jazzy Bridge는 API와 별도 프로세스로 실행합니다.

## 현재 범위

개발 시작용 구조와 CI를 제공합니다. 로봇 제어·업무 기능·모델 추론은 아직 구현하지 않았습니다.
CI 성공은 실로봇·GPU·통합 검증 완료를 의미하지 않습니다.
배포 환경 미정으로 CD와 자동 배포는 설정하지 않았습니다.

## 기본 PyQt 통합관제 UI · FFT1-107
`src/roport_control_server/ui.py`는 실행부와 독립된 표시용 화면입니다. 기본 실행은 **OFFLINE**, `--demo`는 명시적인 모의 데이터입니다. 작업 현황·오류·상태 갱신 시각과 Pinky/OMX 탭을 제공합니다. 작업 승인·출발·취소 버튼은 실행부 연동 전까지 비활성화합니다. UI가 MQTT/UDP/DB에 직접 접근하지 않습니다.

```bash
uv sync --locked --extra ui
uv run --extra ui python -m roport_control_server.ui          # 상태 없음
uv run --extra ui python -m roport_control_server.ui --demo   # 모의 데이터
```

PyQt6 6.12.0을 선택 의존성 `ui`로 고정했습니다([공식 설치 안내](https://www.riverbankcomputing.com/software/pyqt/download)). 팀원의 상세 위젯도 같은 PyQt6 환경을 사용해야 합니다. Linux 실제 창 실행에는 데스크톱과 Qt platform 라이브러리가 필요하며 headless 시험은 `QT_QPA_PLATFORM=offscreen`을 사용하며 Ubuntu의 `libegl1`이 필요합니다(CI에서 설치). 기존 FastAPI health 뼈대는 유지하며 실행부 전환은 사용자 요청으로 보류합니다.

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
