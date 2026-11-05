# 관제 서버

- Python 3.12 / FastAPI / uv를 사용한다. FastAPI 앱은 `src/roport_control_server/adapters/api/app.py`, 웹 화면은 `spatial_assets/`다. 계층과 import 규칙은 README의 구조 절과 `tests/test_layers.py`를 따른다.
- 2026-11-03 팀 합의는 브라우저 웹 UI ↔ FastAPI ↔ 운영 DB ↔ 별도 관제 실행부 1개다. AI 노트북에서 서버를 운영하고 다른 PC에서 접속한다. AI 추론은 별도 프로세스다. API와 실행부는 운영 DB로 연결하고 서비스 WSS는 실행부가 담당한다.
- 현재는 읽기 전용 웹 UI·REST·WebSocket OFFLINE/DEMO 및 MQTT/UDP 모듈만 구현했다. DB·웹에 연결된 운영 실행부·WSS·브라우저 영상 중계·운영 인증은 미구현이다. `executor/pinky_trial.py`는 별도 CLI 시험이며 웹에서 호출하지 않는다. 시험 계약·배포는 `docs/pinky-trial.md`를 따른다. 서버 시작으로 로봇을 연결하지 않는다. DEMO 제어는 브라우저별로 격리하며 영속 복구로 설명하지 않는다.
- Qt/QWidget을 다시 추가하지 않는다. Pinky/OMX 상세 탭은 웹 모듈로 연결하고 UI는 DB/MQTT/UDP에 직접 접근하지 않는다. PyYAML/Pillow는 서버 운영자가 지정한 지도 읽기에만 사용한다. 공개 예시 외 지도/영상은 커밋하지 않는다.
- `adapters/mqtt/`는 Paho·표준 라이브러리·자기 폴더만, `adapters/udp/`는 표준 라이브러리·자기 폴더만 사용한다. Qt·ROS·DB·관제 실행부를 import하지 않아 팀원이 이식할 수 있어야 한다(MQTT는 폴더 단위, UDP는 OMX=protocol+sender·AI=protocol+receiver).
- 모의 장비(`MockExecutor`)와 데모는 `scripts/`에 두며 패키지(wheel)에 넣지 않는다.
- MQTT 통합 검증에는 Mosquitto가 필요하다. CI처럼 `REQUIRE_MQTT_TESTS=1 uv run pytest`로 skip 없는 검증을 수행한다.
- 자산 준비: `npm ci --ignore-scripts`, `npm run prepare:assets`. 브라우저 검사: `npx playwright install chromium`, `npm run test:web`.
- 검사: `uv sync --locked`, `uv run ruff check .`, `uv run ruff format --check .`, `uv run pytest`, `uv build`.
- `/healthz`는 프로세스 생존 확인이다. ROS 연결·장비 준비·작업 완료 판정으로 확장 해석하지 않는다.
- 작업 상태·명령 전달·실패 처리는 호출 흐름과 양쪽 인터페이스를 확인하고 변경한다. timeout·재시도·중복 명령의 처리 결과를 명확히 한다.
- API 테스트는 mock 또는 테스트 환경을 사용한다. 서버 시작만으로 실제 로봇을 연결하거나 구동하지 않는다.
