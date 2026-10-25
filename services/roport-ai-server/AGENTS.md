# AI 추론 서버

- Python 3.12 / FastAPI / uv를 사용한다. 앱은 `src/app.py`다.
- 검사: `uv sync --locked`, `uv run ruff check .`, `uv run ruff format --check .`, `uv run pytest`, `uv build`.
- `/healthz`는 프로세스 생존 확인이다. 모델·GPU·입력 준비가 확인되지 않으면 `/readyz`를 성공으로 바꾸지 않는다.
- 모델 입력·출력 schema, 전처리, 단위, threshold와 모델 버전을 함께 확인한다. 가중치 다운로드나 GPU 의존성을 기본 테스트에 암묵적으로 추가하지 않는다.
- 모델 미설정·잘못된 입력·추론 실패를 성공 응답으로 숨기지 않는다. 장비 제어는 추론 서버에서 자동 실행하지 않는다.
