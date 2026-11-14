# 서비스 예약 ↔ 관제 실행부

서비스는 예약·주문·고객 수령을, 관제는 배차·로봇 명령·선반·통행 상태를 소유한다. 서로의 DB를 직접 수정하지 않는다.

```
고객 예약 API → 서비스 PostgreSQL + outbox
                              ↓ WSS
관제 ServiceTaskStore(inbox + 작업 + outbox)
                              ↓ service_tick
FleetControl → OperationalStore → MQTT → 로봇
                              ↓ 진행/완료 WSS
서비스 control_task_views (업무 상태의 읽기 사본)
```

## 연결

서비스 서버: `SERVICE_CONTROL_ENABLED=true`, `SERVICE_CONTROL_SITE`, `SERVICE_CONTROL_TOKEN`을 설정한다. 관제 서버는 아래 환경변수를 사용한다. 토큰·DB 자격증명은 저장소에 넣지 않는다.

- `ROPORT_DATABASE_URL`: 관제 소유 PostgreSQL 접속 문자열
- `ROPORT_SERVICE_WSS_URL`: `wss://서비스호스트/ws/v1/sites/현장키/control`
- `ROPORT_SERVICE_SITE`: URL의 현장키와 같은 값
- `ROPORT_SERVICE_TOKEN`: 서비스의 전용 토큰

관제 테이블 설치·변경은 명시적으로 실행한다. 기존 설치도 아래 명령을 다시 실행하면 필요한 컬럼을 추가하며, API/실행부 시작은 테이블을 변경하지 않는다.

```bash
uv run python -m roport_control_server.adapters.db.service_tasks
```

`fleet_runtime`은 `ROPORT_DATABASE_URL`이 있으면 운영 상태를 PostgreSQL에 저장하고, WSS URL이 있으면 외부 서비스에 연결한다. 실제 실행 명령과 현장 확인은 통합 실행 설명을 따른다. 파일 journal은 가상 시험에만 쓴다. 운영 저장소는 DB 잠금으로 실행부 한 개만 허용한다.

## 처리 기준

1. 예약 트랜잭션과 서비스 outbox를 함께 저장한다. 요청에는 주문·예약·포장품 목록·보정 목표점 키·예약시각·주문 revision이 들어간다. `package_id` 하나는 실제 포장품 하나이며 quantity는 1이다.
2. 관제는 inbox·작업·접수 응답을 한 트랜잭션으로 저장한 뒤 ACK한다. 동일 message_id 재전송은 같은 작업을 반환한다. 내용이 다른 ID 재사용은 거부한다.
3. 실행부는 같은 task_id/package_id로 자식 작업을 생성한다. 이미 처리한 요청을 다시 읽어도 로봇 작업은 늘어나지 않는다. 작업 상태와 회신 outbox를 함께 저장하고 ACK 전까지 재전송한다.
4. 서비스는 증가한 task_revision만 적용한다. 관제 작업 완료가 주문 수령 완료를 만들지 않는다. 확정된 `receipts`와 `receipt_packages`만 `handover.receipt_recorded`로 전달한다. 관제는 최초 수신 트랜잭션에서 해당 `package_ids` 모두의 운영 상태가 `await_receipt`·`AWAITING_RECEIPT`인지 확인해 DB 전용 `arrival_verified`에 저장한다. 메시지 ACK는 저장 확인이며 수령 승인 의미가 아니다. 조기 수령은 false로 보존하고, 같은 receipt 재전송이나 나중 도착으로 true가 되지 않는다. 도착 후 실제 수령은 새 receipt로 확인해야 한다. 검증된 물품의 복귀만 허용하며, 정상 중복은 복귀 후에도 같은 판정을 반환한다.

기존 receipt에는 최초 도착 확인 근거가 없으므로 명시적 DB 변경 시 `arrival_verified=false`가 적용된다. 기록을 삭제하지 않으며 자동 복귀 근거로 재사용하지 않는다. 아직 수령 대기 중인 작업은 도착 후 새 수령 확인이 필요하다. 원본 메시지 JSON과 MQTT/WSS 필드는 바뀌지 않는다.

## 남은 업무 결정

- `pickup_at`은 고객이 요청한 도착 시각이다. 출발 시각으로 해석하지 않는다. 준비·이동 소요시간을 반영한 출발 정책은 별도 결정이 필요하다.
- 현재 예약 생성 시 잠금된 포장 명세가 없으면 `WAITING_MANIFEST`로 대기한다. 예약 이후 명세를 확정·변경하고 재전달하는 API는 아직 없다. 현재 통합 시험은 명세를 먼저 준비한다.
- 고객 본인 확인·인도 승인·수령 등록 API는 별도 범위다. 기존 서비스 DB는 주문당 수령 행 한 개를 허용한다. 관제는 포장품별 수령 근거를 처리할 수 있으나, 여러 차례 나누어 인도하는 서비스 업무·DB 계약은 아직 확정되지 않았다.
- WSS 재접속은 송수신만 복구한다. 로봇 재시작·결과 미확인은 실행부의 HOLD를 자동 해제하지 않는다.

검증: 격리 PostgreSQL의 `tests/test_service_link.py`, `tests/test_fleet_service.py`; 서비스 Java `ControlExchangeTest`. 실제 WSS 서버·인증서 배포와 실물 움직임은 이 테스트가 대신하지 않는다.
