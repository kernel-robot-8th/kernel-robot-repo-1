# WSS 관제 저장·전송 기반 (합의 전 후보)

**이 PR은 단독 어댑터/저장소만 제공한다.** API와 Fleet 실행부는 아직 이 모듈을 호출하지 않는다.
환경변수를 설정하거나 기존 웹 UI를 시작해도 DB migration·서비스 연결·로봇 구동이 시작되지 않는다.
서비스 Java·Fleet bridge·고객 화면은 후속 PR이다.

계약 비교와 필드 설명: [WSS 후보](../../../interfaces/wss/candidate/README.md).
기존 S1 제안과 호환되지 않으며 운영 연결은 양쪽 계약 합의 후 진행한다.

```text
서비스 시험 peer → ServiceLink → ServiceTaskStore (PostgreSQL)
                    저장 후 ACK ← inbox + 작업 + outbox 원자 저장
                    상태 재전송 ← 미확인 outbox

후속 연결: Fleet 실행부 → OperationalStore / ServiceTaskStore
```

## 책임과 파일

- `adapters/wss/service.py`: 관제가 시작하는 Bearer WebSocket 연결, 수신 검증·ACK·재연결·outbox 재전송.
- `adapters/db/service_tasks.py`: 요청/명세/수령 중복 처리, 진행 이벤트 저장, 실행부 스냅샷 단일 owner.
- `adapters/db/002_service.sql`: 관제 소유 테이블. API/서비스 서버가 직접 수정하지 않는다.
- `contracts/service_wire.py`, `service_policy.py`: 후보 envelope 검증과 배차 전 명세 변경 규칙.

수신 요청과 접수 결과, 진행 변경과 응답은 각각 같은 트랜잭션이다. ACK 전 DB commit을 완료한다.
수령 이력은 최초 도착 근거를 저장하며 조기 수령의 재전송으로 복귀를 승인하지 않는다.
`OperationalStore`는 PostgreSQL advisory lock으로 DB당 한 실행부만 허용한다.
연결이 끊어지면 저장 실패를 호출자에게 전달하며 owner를 자동으로 재획득하지 않는다.
Fleet 연결 PR에서는 저장 실패를 구동 계속의 근거로 쓰지 않아야 한다.

## 단독 검증

저장소 루트가 아닌 `services/roport-control-server`에서 실행한다.
아래 DB는 폐기 가능한 **시험 DB**로 준비한다. 실제 운영 DB/현장 로그를 사용하지 않는다.

```bash
uv sync --locked
export ROPORT_TEST_DATABASE_URL='postgresql://localhost/roport_wss_test'
REQUIRE_REGISTRY_TESTS=1 PYTHONPATH=src uv run pytest -q \
  tests/test_service_link.py tests/test_service_storage.py tests/test_service_manifest.py
```

각 테스트는 UUID 스키마를 만들고 그 스키마만 지운다. WebSocket 시험은 loopback 임시 포트를 사용한다.
모의 서비스와 실제 PostgreSQL 사이에서 재전송/ACK 유실/재연결을 검사하며 실물 장비가 필요 없다.
패키지 wheel에는 SQL과 어댑터가 포함된다. 공용 JSON 예시는 저장소의 `interfaces/wss/candidate/`에 있다.

## 명시적 테이블 준비와 호출 경계

테이블은 프로세스 시작 시 자동 설치하지 않는다. 별도의 검토용 DB에만 접속 문자열을 지정해 실행한다.

```bash
# ROPORT_DATABASE_URL은 로컬 비밀 설정에서 주입한다.
uv run python -m roport_control_server.adapters.db.service_tasks
```

기존 스냅샷/작업을 삭제하지 않으며 반복 실행 시 누락된 `arrival_verified`, `dispatch_locked`를 추가한다.
기존 수령 이력은 도착 근거가 없으므로 복귀 허가로 쓰지 않는다. 버전별 운영 migration은 후속 합의 범위다.

후속 실행부는 `OperationalStore(dsn)` 한 개와 `ServiceTaskStore(dsn)`를 소유하고,
`ServiceLink(url, token, site_id, store).run(stop_event)`를 별도 asyncio 작업으로 실행한다.
`ServiceTaskStore.acknowledge(message_id, site_id)`는 인증된 site 범위의 ACK만 반영한다.
4001 종료·DB 오류는 실행부가 관찰해야 한다. 이 PR은 운영 실행 CLI를 추가하지 않는다.

미검증: 실제 서비스 Java와의 상호운용, 운영 TLS/프록시/인증, 실예약·실물 구동.
