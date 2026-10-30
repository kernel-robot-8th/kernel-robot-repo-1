# 관제 서버

- Python 3.12 / FastAPI / uv를 사용한다. 앱은 `src/roport_control_server/app.py`다.
- FastAPI는 기존 health 뼈대이며 최종 구조는 PyQt ↔ 별도 관제 실행부 직접 연결이다. 전환은 후속 PR에서 한다.
- `communication/`은 Paho와 표준 라이브러리만 사용한다. Qt·ROS·DB·관제 실행부를 import하지 않아 팀원이 폴더 단위로 이식할 수 있어야 한다.
- MQTT 통합 검증에는 Mosquitto가 필요하다. CI처럼 `REQUIRE_MQTT_TESTS=1 uv run pytest`로 skip 없는 검증을 수행한다.
- 검사: `uv sync --locked`, `uv run ruff check .`, `uv run ruff format --check .`, `uv run pytest`, `uv build`.
- `/healthz`는 프로세스 생존 확인이다. ROS 연결·장비 준비·작업 완료 판정으로 확장 해석하지 않는다.
- 작업 상태·명령 전달·실패 처리는 호출 흐름과 양쪽 인터페이스를 확인하고 변경한다. timeout·재시도·중복 명령의 처리 결과를 명확히 한다.
- API 테스트는 mock 또는 테스트 환경을 사용한다. 서버 시작만으로 실제 로봇을 연결하거나 구동하지 않는다.
