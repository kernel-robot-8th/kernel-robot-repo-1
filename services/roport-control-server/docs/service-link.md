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

## 임시 적용 정책과 교체 경계

- `pickup_at`은 고객 도착 희망 시각이다. `fleet.json`의 `service_policy.dispatch_lead_s`(기본 300초)만큼 앞서 포장품 작업을 생성한다. 그 전에는 요청만 저장하며, 생성 직전 DB의 `dispatch_locked`를 잠가 목록 갱신과 경합하지 않는다. 300초는 실측한 소요시간이 아니다. 출발 계산은 `contracts/service_policy.py`에 모아 두었으며 실제 준비·이동시간 계산으로 바꿔도 MQTT 형식은 그대로다. 입고 `prepare_order`는 즉시 대기열에 들어간다.
- 포장 담당 코드가 서비스 DB의 활성 포장품과 `manifest_locked_at`을 확정한 후 `POST /api/pickups/{pickupId}/manifest-refresh`를 호출한다. 본인 예약에 대해서만 저장된 명세를 전송하며, 요청 본문에 임의 package ID를 받지 않는다. 목록이 같으면 재전송하지 않는다. 변경하면 주문 revision과 outbox를 함께 저장한다. 이는 포장 업무 자체를 구현한 API가 아니다.
- 관제는 **자식 작업 생성 전**에만 새 revision의 포장 목록을 적용한다. 출발 예약 후 변경·늦은 이전 revision은 원래 명세를 유지하고 `task.progress.progress.manifest_change`에 `accepted=false`와 이유를 저장·회신한 뒤 ACK한다. HTTP `submitted=true`는 전송 대기 등록이며 관제 적용 성공이 아니다. 목적지/예약시각 변경은 이번 명세 갱신의 범위가 아니다.
- 기존 서비스 QR·예약번호 검증을 재사용한다. PR #81 기준 QR 10분 만료, 카트별 1분 내 실패 5회 제한을 적용했다. `POST /cart-api/receipts`는 `{ "verificationId": "검증 결과 ID", "confirmationRequestId": "이번 확인의 고유 ID" }`를 받는다. 최근 5분 내 성공한 동일 카트 검증, 유효 QR(사용한 경우), 관제의 해당 카트/포장품 `await_receipt` 진행 사본을 확인한 후 명시적 수령을 기록한다. QR 조회나 로봇 도착만으로 수령을 만들지 않는다.
- 한 카트의 순차 운송을 위해 `V3__package_deliveries.sql`에서 주문당 receipt 1개 제약만 제거했다. 포장품별 중복 수령과 확인 요청 ID 중복은 기존 제약으로 막는다. 현재 카트에 배정된 포장품만 기록하고, 활성 포장품 모두 수령되었을 때 주문 RECEIVED/예약 CLOSED로 확정한다. 동일 확인 요청은 동일 receipt를 반환한다(`orderComplete`는 조회 시점의 주문 상태). 다른 운송 회차에는 새 확인 요청 ID를 쓴다.

관제의 최초 수령 도착 검증(`arrival_verified`)은 그대로 남는다. 서비스의 진행 사본이 오래되어 실제 관제가 HOLD라면 수령 기록이 자동 복귀 허가로 변하지 않는다. 이러한 불일치는 기록을 삭제하지 않고 운영자가 확인해야 한다. 고객/카트 인증은 기존 개발 전용 방식이며 운영 인증·실시간 인도 승인 계약은 별도다.

서비스 정책은 `service_policy.py`, 고객 수령은 서비스 `HandoverService`, 전송은 WSS adapter로 분리했다. PR #82의 S1은 요청 거절용 단계 제안이므로 현재 예약 처리 경로에 섞지 않았다. 그 계약을 확정할 때 WSS 변환과 계약 테스트를 함께 교체한다. MQTT 토픽·최소 메시지 스키마는 변경하지 않았다.

남은 현장 확정: 실제 준비/이동 시간, 포장 완료 알림의 호출 주체, 운영 고객/카트 인증, 최종 물품 크기 판정 주체. 크기는 기존 OMX3 상차 결과를 사용하고, 현재 실물 factory는 운영자가 정한 `package_sizes`로 공급한다. unknown을 임의로 small/large로 바꾸지 않는다.

검증: 격리 PostgreSQL의 `tests/test_service_link.py`, `tests/test_fleet_service.py`, `tests/test_service_policy.py`; 서비스 Java `ControlExchangeTest`, `QrVerificationApiTest`, `VerifyAttemptLimiterTest`. WSS 공개 배포·실물 움직임·실제 고객 인도는 별도 검증한다.
