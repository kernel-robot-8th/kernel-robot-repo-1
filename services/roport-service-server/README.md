# roport-service-server

RO:PORT · GATE CREW — 회원·주문·예약·수령 처리 API

## 기술 스택

Java 21 / Spring Boot 4.1.2 / Gradle(wrapper). 업무 데이터는 PostgreSQL(JDBC·Flyway)입니다.

## 현재 범위

고객 웹 첫 슬라이스: 세션 로그인, 주문·수령 위치·항공편 조회, 예약 가능 시간, 수령 예약 생성(요청 ID 없음, 주문당 활성 예약 1개 — 두 번째 요청·재전송은 409 `ACTIVE_PICKUP_EXISTS`)과 조회(`/api/**`, 오류는 ProblemDetail + `code`).
업무 code가 없는 오류는 상태별 일반 code를 씁니다: 404 `NOT_FOUND`, 405 `METHOD_NOT_ALLOWED`, 406 `NOT_ACCEPTABLE`, 409 `CONFLICT`, 415 `UNSUPPORTED_MEDIA_TYPE`, 그 밖의 4xx `BAD_REQUEST`, 5xx `INTERNAL_ERROR`(내부 정보는 응답에 넣지 않고 서버 로그에만 남깁니다).
예약은 `REQUESTED`(예약 접수됨 – 현장 배정 전)까지만 만듭니다. 카트 확보·관제 수락·수령 완료가 아닙니다. 관제 WSS는 명시적으로 활성화할 수 있습니다. 예약과 같은 트랜잭션에 `task.requested` outbox를 저장하고, 관제의 접수·진행·완료는 별도 읽기 사본(`control_task_views`)으로 받습니다.

예약 QR·카트 확인(FFT1-178): `POST /api/pickups/{pickupId}/qr-token`은 본인 활성 예약에 새 QR 토큰을 발급하고 이전 토큰을 폐기합니다(원문은 응답에만, DB에는 SHA-256 해시만). 만료는 발급 후 `service.qr.ttl`(기본 10분)이며 응답의 `expiresAt`으로 알려 줍니다. 이 변경 전에 발급돼 `expires_at`이 비어 있는 토큰(개발 DB에만 있음)은 예전처럼 만료되지 않습니다.
`POST /cart-api/verifications`는 QR 토큰 또는 예약번호로 예약을 찾아 `verifications`에 결과를 기록합니다(새 확인 201, 같은 `verificationRequestId` 재전송 200, 다른 내용 409 `IDEMPOTENCY_CONFLICT`, 예약 없음 404 `RESERVATION_NOT_FOUND`). 이미 쓴 요청 ID에 없는 값을 보내면 404보다 409가 우선합니다.
카트별로 확인 실패(예약 없음·거절)가 `service.verify.failure-window`(기본 1분) 안에 `service.verify.max-failures`(기본 5회)에 이르면 새 확인 요청은 429 `TOO_MANY_ATTEMPTS`입니다(FFT1-182). 429에는 `Retry-After`(초)를 붙입니다. 기록은 서버 메모리에만 두며, 카트 ID를 위조할 수 있는 동안에는 ID를 바꿔 제한을 피하거나 실제 패드의 ID로 실패를 쌓아 그 패드를 잠글 수 있습니다. 같은 요청 ID 재전송 때 QR이 만료·폐기됐으면 저장된 결과만 돌려주고 최신 예약 정보는 넣지 않습니다.
**확인은 인도 승인이 아닙니다.** verification 응답의 `cartAssignmentChecked=false`는 유지하며, 별도 수령 API에서 현재 카트 배정과 도착 상태를 대조합니다.
카트 인증도 개발 전용입니다. `X-Cart-Id` 헤더만 확인하며(비밀값 없음) 기본으로 꺼져 있어(모든 `/cart-api/**` 401) 로컬에서만 `SERVICE_DEV_CART=true`로 켭니다.

**인증은 개발 전용입니다.** `login_id`만으로 로그인하는 세션 쿠키이며 비밀번호·Spring Security가 없으므로 공개 배포하지 않습니다.
로그인은 기본으로 꺼져 있어(모든 `/api/**` 401) 로컬에서만 `SERVICE_DEV_LOGIN=true`로 켭니다.
CI 성공은 실로봇·GPU·통합 검증 완료를 의미하지 않습니다.
배포 환경 미정으로 CD와 자동 배포는 설정하지 않았습니다.

## 관련 문서

이 디렉토리는 단일 저장소의 구성요소입니다. 아래 명령은 이 디렉토리에서 실행합니다.

- [프로젝트 개요](https://github.com/kernel-robot-8th/kernel-robot-repo-1)
- [요구사항·설계 문서](../../docs/README.md)
- [Jira](https://fp-team1.atlassian.net/jira/projects/FFT1)

PR에 관련 Jira 키, 변경 내용, 검증 결과와 미검증 항목을 기록합니다.
실로봇 적용 전 장비·설정·캘리브레이션, 안전 정지·복구 절차를 확인합니다.

## 개발 방법

Java 21을 사용하며 Gradle은 저장소의 wrapper(`./gradlew`)로 실행합니다.

```bash
./gradlew build
./gradlew bootRun
```

DB 접속은 환경변수 `SERVICE_DB_URL`·`SERVICE_DB_USER`·`SERVICE_DB_PASSWORD`로 줍니다. 앱 시작 시 Flyway가 `db/migration`을 적용합니다.
`./gradlew build`의 테스트도 이 DB를 씁니다. 격리된 테스트 DB만 지정합니다(실행마다 고유 ID의 가짜 행을 남기며, DB 이름이 `_test`로 끝나지 않으면 Flyway 적용 전에 테스트가 실패합니다).

개발용 더미 데이터(가짜 값, 자동 적용 안 됨, 로그인 `demo1`·`demo2`):

```bash
psql -v ON_ERROR_STOP=1 -d <개발 DB> -f src/main/resources/db/dev/seed.sql
```

상태 확인: `http://127.0.0.1:8080/actuator/health`.


## 관제 WSS 연결

서비스 설정은 `SERVICE_CONTROL_ENABLED=true`, `SERVICE_CONTROL_SITE`, `SERVICE_CONTROL_TOKEN`입니다. 토큰은 별도로 발급해 환경변수로만 전달합니다. 관제가 `/ws/v1/sites/{site}/control`로 연결하며 서비스는 사이트당 하나의 연결을 허용하고 새 연결로 교체하면 이전 연결을 4001로 닫습니다. 외부 연결은 HTTPS/WSS가 필요합니다(로컬 TLS 프록시 뒤에서 운영할 때도 외부 TLS는 배포 책임입니다). HTTP 시험은 loopback에서만 허용합니다.

예약 생성의 `task.requested`에는 주문·예약·잠금 완료된 활성 포장품 목록(포장품 하나당 quantity=1)·`pickup_locations.control_point_ref`·예약시각이 들어갑니다. 관제의 보정된 목표점 키와 `control_point_ref`를 맞춰야 합니다. 빈 포장품 목록은 관제에서 `WAITING_MANIFEST`로 남으며 로봇을 출발시키지 않습니다. `POST /api/pickups/{pickupId}/manifest-refresh`는 본인 예약의 DB에 확정된 포장 명세를 다시 전달합니다. 포장 업무 자체는 별도이며, 관제는 작업 생성 전에만 새 목록을 적용합니다. 응답의 `submitted=true`는 outbox 등록이며 실제 적용은 관제 회신으로 확인합니다.

양쪽은 처리 내용을 DB에 commit한 뒤 `message.ack`를 보냅니다. ACK를 잃거나 연결이 끊기면 동일 메시지를 다시 보내고, `message_id` 중복을 제거합니다. 관제 상태는 `task_revision`이 증가할 때만 갱신하며 완료 상태를 과거 진행으로 되돌리지 않습니다. 완료 메시지는 `orders.business_status`나 고객 수령을 바꾸지 않습니다.

`POST /cart-api/receipts`는 `{ "verificationId": "검증 결과 ID", "confirmationRequestId": "확인 요청의 고유 ID" }`를 받습니다. 같은 카트의 5분 이내 성공 검증(`service.handover.verification-age`), QR 유효성, 관제 진행 사본의 카트 배정/도착을 확인하고 해당 포장품 수령을 기록합니다. HTTP 200은 `{receiptId, packageIds, orderComplete, followMode}`이며 같은 요청은 동일 receipt를 반환합니다. 실제 수령 의사 확인 후에만 호출합니다. 확정된 수령 근거는 `handover.receipt_recorded`로 관제에 전달하고, 관제는 자체 최신 도착 상태를 다시 검사합니다.

검증: `./gradlew --no-daemon build`는 격리 PostgreSQL에서 예약/outbox 원자성, 실제 WebSocket 인증·저장 후 ACK, 재전송·순서 역전·고객 수령 분리를 검사합니다.

`V3__package_deliveries.sql`은 주문당 receipt 1개 제약을 제거해 카트가 여러 번 나누어 운송할 수 있게 합니다. 기존 receipt 행은 보존하며, package_id·confirmation_request_id 중복 금지는 유지합니다. 마지막 활성 포장품의 수령 시에만 주문 RECEIVED/예약 CLOSED가 됩니다. 관제와 서비스 상태가 어긋나면 자동 재처리로 복귀를 승인하지 않으므로 운영자 확인이 필요합니다. 정책/전송의 교체 경계와 남은 현장 항목은 [관제 서비스 연결](../roport-control-server/README.md#서비스-wss-관제-기반-합의-전-후보)을 참조하세요.

## 로컬 관제·고객 화면 통합

[통합 가상 시험 실행 안내](../roport-control-server/docs/service-web-local-test.md). 고객 상태는 `GET /api/pickups/{pickupId}/progress`로 소유권 검사 후 조회한다. QR/예약번호 확인 결과의 `verificationId`로 `/cart-api/receipts`에 수령을 명시 요청한다. 도착 전 수령·다른 카트·중복 수령은 서버에서 판정한다.

## 추종 방식 선택·조작 (FFT1-207)

수령 요청에는 `followMode: "none" | "marker" | "appearance"`를 선택해 넣는다. `marker`만 `followCard: 35..49`가 필요하며 다른 모드에 카드 번호를 넣으면 400이다. 모드를 생략한 이전 요청은 카드가 있으면 marker, 없으면 none으로 유지하고 기존 WSS payload 형태를 보존한다. 명시한 marker/appearance는 현재 카트의 관제 capability를 다시 확인한다. `V6__receipt_follow_modes.sql`이 저장 필드를 추가한다.

| API | 결과 |
| --- | --- |
| `GET /cart-api/follow/capabilities?verificationId=...` | 해당 카트의 신선한 도착 상태에서 `modes` 반환. 미지원 방식은 수령 요청에서도 거절 |
| `GET /cart-api/follow/{receiptId}` | mode·stage·status·revision·allowedActions·stale·registration·lastRequest로 새로고침 복원 |
| `POST /cart-api/follow/{receiptId}/actions` | `{requestId, expectedRevision, operation}`을 durable WSS outbox에 저장. 202 `{accepted:true,requestId}`는 접수이며 실행 성공 아님 |
| `GET /cart-api/follow/{receiptId}/frames/{frameRef}` | 소유 카트·현재 세션·프레임·만료를 확인한 스냅샷 프록시. 캐시 금지 |

`operation`은 capture/select/release/pause/resume/return이다. select에만 `frameRef`와 `candidateId` 또는 정규화 `[x1,y1,x2,y2]` `box` 하나를 넣는다. 임의 `target_ref`나 후보와 영역을 동시에 보낼 수 없다. 등록 성공은 관제의 로봇 결과를 받아 표시한다. 재전송은 같은 requestId·같은 본문을 사용하며 다른 본문은 409다. 서비스 관제 사본이 10초, 로봇 관측값이 3초를 넘거나 작업이 RUNNING이 아니면 새 조작을 거절한다. 재접속은 상태 조회만 수행하고 조작을 자동 재개하지 않는다.

프레임 연결은 `SERVICE_FOLLOW_PREVIEW_BASE_URL`에 관제 HTTP(S) origin을, `SERVICE_FOLLOW_PREVIEW_TOKEN`에 전용 토큰을 넣는다(생략 시 `SERVICE_CONTROL_TOKEN`). 토큰은 관제의 `ROPORT_FOLLOW_PREVIEW_TOKEN`과 맞춘다. HTTP는 loopback만 허용하며 원격 관제는 HTTPS로 연결한다. 경로는 서버가 `/api/fleet/follow/{receiptId}/frames/{frameRef}`로 고정한다. 미설정은 503이며 외부 URL·redirect는 허용하지 않는다. JPEG/PNG만 최대 2MiB, 연결 1.5초·읽기 2.5초 제한으로 읽는다. 카트 인증은 위의 개발 전용 설정 그대로이므로 공개 운영 인증을 구현한 것으로 보지 않는다.
