# 관제 서버

- Python 3.12 / FastAPI / uv를 사용한다. FastAPI 앱은 `src/roport_control_server/adapters/api/app.py`, 웹 화면은 `spatial_assets/`다. 계층과 import 규칙은 README의 구조 절과 `tests/test_layers.py`를 따른다.
- 현재 연결은 브라우저 웹 UI ↔ FastAPI ↔ Unix socket ↔ 별도 관제 실행부 1개다. 실행부가 운영 PostgreSQL과 서비스 WSS/MQTT를 소유한다. API가 별도 DB 상태를 만들거나 MQTT를 직접 발행하지 않는다. AI 추론은 별도 프로세스다.
- OFFLINE/DEMO/TRIAL/FLEET를 제공한다. `executor/fleet_runtime.py`는 요청 기반 역할·선반·입출고·N12 왕복을 처리하며 실제 운영 상태는 PostgreSQL, 가상 시험은 파일 journal을 사용한다. `executor/pinky_trial.py`의 구간 시험은 유지한다. 각 실행부를 같은 기체에 중복 연결하지 않는다. 시작만으로 구동하지 않는다. `docs/field-control.md`와 `docs/service-link.md`의 남은 현장·업무 결정을 구현 완료로 설명하지 않는다.
- Qt/QWidget을 다시 추가하지 않는다. Pinky/OMX 상세 탭은 웹 모듈로 연결하고 UI는 DB/MQTT/UDP에 직접 접근하지 않는다. PyYAML/Pillow는 서버 운영자가 지정한 지도 읽기에만 사용한다. 공개 예시 외 지도/영상은 커밋하지 않는다.
- `adapters/mqtt/`는 Paho·표준 라이브러리·자기 폴더만, `adapters/udp/`는 표준 라이브러리·자기 폴더만 사용한다. Qt·ROS·DB·관제 실행부를 import하지 않아 팀원이 이식할 수 있어야 한다(MQTT는 폴더 단위, UDP는 OMX=protocol+sender·AI=protocol+receiver).
- 모의 장비(`MockExecutor`)와 데모는 `scripts/`에 두며 패키지(wheel)에 넣지 않는다.
- MQTT 통합 검증에는 Mosquitto가 필요하다. CI처럼 `REQUIRE_MQTT_TESTS=1 uv run pytest`로 skip 없는 검증을 수행한다.
- DB 드라이버는 `adapters/db/`만 import한다. 등록 검증에는 격리 PostgreSQL의 `ROPORT_TEST_DATABASE_URL`과 `REQUIRE_REGISTRY_TESTS=1`이 필요하다. 테스트는 UUID 스키마를 만들고 자신이 만든 스키마만 삭제한다. 운영 DB를 테스트에 사용하지 않는다. 초기화는 명시적 CLI로만 실행하며 개발 모드는 `sim-` 기체만 허용한다.
- 자산 준비: `npm ci --ignore-scripts`, `npm run prepare:assets`. 브라우저 검사: `npx playwright install chromium`, `npm run test:web`.
- 검사: `uv sync --locked`, `uv run ruff check .`, `uv run ruff format --check .`, `uv run pytest`, `uv build`.
- `/healthz`는 프로세스 생존 확인이다. ROS 연결·장비 준비·작업 완료 판정으로 확장 해석하지 않는다.
- 작업 상태·명령 전달·실패 처리는 호출 흐름과 양쪽 인터페이스를 확인하고 변경한다. timeout·재시도·중복 명령의 처리 결과를 명확히 한다.
- API 테스트는 mock 또는 테스트 환경을 사용한다. 서버 시작만으로 실제 로봇을 연결하거나 구동하지 않는다.

- `executor/plans.py`의 순차 계획을 기존 시험과 UI 구간/작업·출고 요청 시험이 공유한다. 저장 정의는 journal 옆 `scenario-plans.json`, 실행본은 journal에 고정한다. 기존 구간 시험은 정차선 노드에서만 route를 끝내며, 중간 U턴·정차선 통과·line-only의 Nav2 추가를 거부한다. 이 구간 시험을 별도의 fleet WSS/재고/OMX 구현과 혼동하지 않는다. fleet는 기체별 활성 명령과 남은 노드·양방향 간선을 예약해 동시 주행한다. 앞 IR 감지 뒤 들어온 간선은 다음 노드까지 보호한다. 실측 `node_positions`와 검증된 `node_clearance_m`가 있으면 모든 과거 노드를 남은 경로와의 최소거리로 보호·해제해 짧은 연속 구간과 해제 후 재예약을 방지한다. 설정이 없으면 기존 한 구간 보호를 유지한다. 이 규칙은 활성 명령의 예약이며 명령 완료 후 전역 차체 간격 보장을 의미하지 않는다. 부분 경로 도착은 업무 완료로 처리하지 않는다. 중간 교차점 정차는 Pinky driver의 내부 hold_at_goal 경로를 사용한다. OMX host 작업은 한 개씩 실행한다.
- UI 시나리오 선택은 공통 `two_pinky` 설정과 기존 binding을 유지한 채 1대/2대/스마트카트 계획을 적용한다. `allowed_robots`는 선택 가능 범위, `robots`는 현재 계획 참여 기체다. 참여 기체만 폴링/준비/timeout/점유 검사하며 비참여 기체는 현장에서 통로 밖에 둔다. 선택/노드 편집은 구동하지 않고 READY/COMPLETE에서 검증·적용 후 별도 출발한다. HOLD는 UI로 해제하지 않는다.

- 실물 코드 기반 가상 환경은 `tools/run_virtual_trial.py` → `scripts/virtual_pinky.py`다. 현재 checkout의 Pinky TrialDriver/TrialLogic/RouteExecutor와 팀 2D Sim을 재사용한다. Nav2·lease는 모사이며 물리 검증으로 표현하지 않는다. loopback·새 binding·이력 보존과 가상 전용 초기화 경계를 유지한다.

- 서비스 작업 시험은 `executor/plans.py`의 `service_plan`으로 기존 입고/출고 단계를 생성하고, UI는 서버 미리보기 ID로 적용한 뒤 별도로 출발한다. 후보 기체 상태 조회는 읽기 전용이며 적용 전에는 참여 기체/구동을 바꾸지 않는다. 운영 주문·재고·WSS 계약으로 설명하지 않는다.
- `executor/omx_trial.py`는 PR45 `roport-omx-task-v1` COOPERATIVE 시험 연동이다. 단일 owner가 처리하며 UI/MQTT 완료를 정차 세션별 한 번만 반영한다. 팔 복귀 근거를 재고/수령 확정으로 바꾸지 않는다. OMX3는 수동이며 `--omx mock`만 가상 실행기에서 모의 Host를 띄운다. 테스트가 OMX 소스를 직접 재사용하므로 양쪽 계약 변경 시 관제 CI도 실행한다.
