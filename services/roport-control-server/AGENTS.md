# 관제 서버

- Python 3.12 / FastAPI / uv를 사용한다. 앱은 `src/roport_control_server/app.py`다.
- 검사: `uv sync --locked`, `uv run ruff check .`, `uv run ruff format --check .`, `uv run pytest`, `uv build`.
- `/healthz`는 프로세스 생존 확인이다. ROS 연결·장비 준비·작업 완료 판정으로 확장 해석하지 않는다.
- 작업 상태·명령 전달·실패 처리는 호출 흐름과 양쪽 인터페이스를 확인하고 변경한다. timeout·재시도·중복 명령의 처리 결과를 명확히 한다.
- API 테스트는 mock 또는 테스트 환경을 사용한다. 서버 시작만으로 실제 로봇을 연결하거나 구동하지 않는다.
