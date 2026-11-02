# 관제 서버

- Python 3.12 / FastAPI / uv를 사용한다. FastAPI 앱은 `src/roport_control_server/adapters/api/app.py`, PyQt 화면은 `adapters/ui/`다. 계층과 import 규칙은 README의 구조 절과 `tests/test_layers.py`를 따른다.
- 통합관제 Tech Spec v4 기준 구성은 PyQt ↔ FastAPI ↔ 운영 DB ↔ 관제 실행부(별도 프로세스)이며 API와 실행부는 운영 DB로만 연결한다. 서비스 WSS는 유지한다. 현재 FastAPI는 `/healthz` 뼈대뿐이고 운영 DB 연결과 관제 실행부는 아직 구현하지 않았다.
- `adapters/mqtt/`는 Paho·표준 라이브러리·자기 폴더만, `adapters/udp/`는 표준 라이브러리·자기 폴더만 사용한다. Qt·ROS·DB·관제 실행부를 import하지 않아 팀원이 이식할 수 있어야 한다(MQTT는 폴더 단위, UDP는 OMX=protocol+sender·AI=protocol+receiver).
- 모의 장비(`MockExecutor`)와 데모는 `scripts/`에 두며 패키지(wheel)에 넣지 않는다.
- MQTT 통합 검증에는 Mosquitto가 필요하다. CI처럼 `REQUIRE_MQTT_TESTS=1 uv run pytest`로 skip 없는 검증을 수행한다.
- 검사: `uv sync --locked`, `uv run ruff check .`, `uv run ruff format --check .`, `uv run pytest`, `uv build`.
- `/healthz`는 프로세스 생존 확인이다. ROS 연결·장비 준비·작업 완료 판정으로 확장 해석하지 않는다.
- 작업 상태·명령 전달·실패 처리는 호출 흐름과 양쪽 인터페이스를 확인하고 변경한다. timeout·재시도·중복 명령의 처리 결과를 명확히 한다.
- API 테스트는 mock 또는 테스트 환경을 사용한다. 서버 시작만으로 실제 로봇을 연결하거나 구동하지 않는다.
