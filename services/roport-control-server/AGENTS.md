# 관제 서버

- Python 3.12 / FastAPI / uv를 사용한다. FastAPI 앱은 `src/roport_control_server/adapters/api/app.py`, 웹 화면은 `spatial_assets/`다. 계층과 import 규칙은 README의 구조 절과 `tests/test_layers.py`를 따른다.
- 2026-11-03 팀 합의는 브라우저 웹 UI ↔ FastAPI ↔ 운영 DB ↔ 별도 관제 실행부 1개다. AI 노트북에서 서버를 운영하고 다른 PC에서 접속한다. AI 추론은 별도 프로세스다. API와 실행부는 운영 DB로 연결하고 서비스 WSS는 실행부가 담당한다.
- 현재는 OFFLINE/DEMO와 명시적으로 설정한 TRIAL 웹 UI를 제공한다. DB 구현은 보류하며 WSS·브라우저 영상 중계·운영 인증은 후속이다. `executor/pinky_trial.py`는 별도 프로세스 1개로 MQTT를 소유하고 API는 Unix socket으로 시험 조작·상태를 주고받는다. CLI와 웹 소유자를 중복 실행하지 않는다. 파일 journal은 시험용이며 운영 DB로 취급하지 않는다. 시험 계약·배포는 `docs/pinky-trial.md`를 따른다. 서버 시작으로 로봇을 연결하지 않는다. DEMO 제어는 브라우저별로 격리하며 영속 복구로 설명하지 않는다.
- Qt/QWidget을 다시 추가하지 않는다. Pinky/OMX 상세 탭은 웹 모듈로 연결하고 UI는 DB/MQTT/UDP에 직접 접근하지 않는다. PyYAML/Pillow는 서버 운영자가 지정한 지도 읽기에만 사용한다. 공개 예시 외 지도/영상은 커밋하지 않는다.
- `adapters/mqtt/`는 Paho·표준 라이브러리·자기 폴더만, `adapters/udp/`는 표준 라이브러리·자기 폴더만 사용한다. Qt·ROS·DB·관제 실행부를 import하지 않아 팀원이 이식할 수 있어야 한다(MQTT는 폴더 단위, UDP는 OMX=protocol+sender·AI=protocol+receiver).
- 모의 장비(`MockExecutor`)와 데모는 `scripts/`에 두며 패키지(wheel)에 넣지 않는다.
- MQTT 통합 검증에는 Mosquitto가 필요하다. CI처럼 `REQUIRE_MQTT_TESTS=1 uv run pytest`로 skip 없는 검증을 수행한다.
- 자산 준비: `npm ci --ignore-scripts`, `npm run prepare:assets`. 브라우저 검사: `npx playwright install chromium`, `npm run test:web`.
- 검사: `uv sync --locked`, `uv run ruff check .`, `uv run ruff format --check .`, `uv run pytest`, `uv build`.
- `/healthz`는 프로세스 생존 확인이다. ROS 연결·장비 준비·작업 완료 판정으로 확장 해석하지 않는다.
- 작업 상태·명령 전달·실패 처리는 호출 흐름과 양쪽 인터페이스를 확인하고 변경한다. timeout·재시도·중복 명령의 처리 결과를 명확히 한다.
- API 테스트는 mock 또는 테스트 환경을 사용한다. 서버 시작만으로 실제 로봇을 연결하거나 구동하지 않는다.

- `executor/plans.py`의 순차 계획을 기존 시험과 UI 구간/작업·출고 요청 시험이 공유한다. 저장 정의는 journal 옆 `scenario-plans.json`, 실행본은 journal에 고정한다. 현재 로봇 주행기와 동일하게 정차선 노드에서만 route를 끝내며, 중간 U턴·정차선 통과·line-only의 Nav2 추가를 거부한다. 실제 WSS/재고/OMX 공동 작업·동시 스케줄링을 구현 완료로 설명하지 않는다.
- UI 시나리오 선택은 공통 `two_pinky` 설정과 기존 binding을 유지한 채 1대/2대/스마트카트 계획을 적용한다. `allowed_robots`는 선택 가능 범위, `robots`는 현재 계획 참여 기체다. 참여 기체만 폴링/준비/timeout/점유 검사하며 비참여 기체는 현장에서 통로 밖에 둔다. 선택/노드 편집은 구동하지 않고 READY/COMPLETE에서 검증·적용 후 별도 출발한다. HOLD는 UI로 해제하지 않는다.

- 실물 코드 기반 가상 환경은 `tools/run_virtual_trial.py` → `scripts/virtual_pinky.py`다. 현재 checkout의 Pinky TrialDriver/TrialLogic/RouteExecutor와 팀 2D Sim을 재사용한다. Nav2·lease는 모사이며 물리 검증으로 표현하지 않는다. loopback·새 binding·이력 보존과 가상 전용 초기화 경계를 유지한다.

- 서비스 작업 시험은 `executor/plans.py`의 `service_plan`으로 기존 입고/출고 단계를 생성하고, UI는 서버 미리보기 ID로 적용한 뒤 별도로 출발한다. 후보 기체 상태 조회는 읽기 전용이며 적용 전에는 참여 기체/구동을 바꾸지 않는다. 운영 주문·재고·WSS 계약으로 설명하지 않는다.
- `executor/omx_trial.py`는 PR45 `roport-omx-task-v1` COOPERATIVE 시험 연동이다. 단일 owner가 처리하며 UI/MQTT 완료를 정차 세션별 한 번만 반영한다. 팔 복귀 근거를 재고/수령 확정으로 바꾸지 않는다. OMX3는 수동이며 `--omx mock`만 가상 실행기에서 모의 Host를 띄운다. 테스트가 OMX 소스를 직접 재사용하므로 양쪽 계약 변경 시 관제 CI도 실행한다.
