# roport-service-server

RO:PORT · GATE CREW — 회원·주문·예약·수령 처리 API

## 기술 스택

Java 21 / Spring Boot 4.1.2 / Gradle(wrapper). 업무 데이터는 PostgreSQL(JDBC·Flyway)입니다.

## 현재 범위

고객 웹 첫 슬라이스: 세션 로그인, 주문·수령 위치·항공편 조회, 예약 가능 시간, 수령 예약 생성(요청 ID 없음, 주문당 활성 예약 1개 — 두 번째 요청·재전송은 409 `ACTIVE_PICKUP_EXISTS`)과 조회(`/api/**`, 오류는 ProblemDetail + `code`).
업무 code가 없는 오류는 상태별 일반 code를 씁니다: 404 `NOT_FOUND`, 405 `METHOD_NOT_ALLOWED`, 406 `NOT_ACCEPTABLE`, 409 `CONFLICT`, 415 `UNSUPPORTED_MEDIA_TYPE`, 그 밖의 4xx `BAD_REQUEST`, 5xx `INTERNAL_ERROR`(내부 정보는 응답에 넣지 않고 서버 로그에만 남깁니다).
예약은 `REQUESTED`(예약 접수됨 – 현장 배정 전)까지만 만듭니다. 카트 확보·관제 수락·수령 완료가 아닙니다. 관제 WSS는 명시적으로 활성화할 수 있습니다. 예약과 같은 트랜잭션에 `task.requested` outbox를 저장하고, 관제의 접수·진행·완료는 별도 읽기 사본(`control_task_views`)으로 받습니다.

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

서비스 설정은 `SERVICE_CONTROL_ENABLED=true`, `SERVICE_CONTROL_SITE`, `SERVICE_CONTROL_TOKEN`입니다. 토큰은 별도로 발급해 환경변수로만 전달합니다. 관제가 `/ws/v1/sites/{site}/control`로 연결하며 서비스는 사이트당 하나의 연결을 허용합니다. 외부 연결은 HTTPS/WSS가 필요합니다(로컬 TLS 프록시 뒤에서 운영할 때도 외부 TLS는 배포 책임입니다). HTTP 시험은 loopback에서만 허용합니다.

예약 생성의 `task.requested`에는 주문·예약·잠금 완료된 활성 포장품 목록(포장품 하나당 quantity=1)·`pickup_locations.control_point_ref`·예약시각이 들어갑니다. 관제의 보정된 목표점 키와 `control_point_ref`를 맞춰야 합니다. 빈 포장품 목록은 관제에서 `WAITING_MANIFEST`로 남으며 로봇을 출발시키지 않습니다. **예약 이후 포장 명세를 확정·변경하는 업무 API와 재전달은 아직 미구현**이므로 현재 통합 시험에서는 명세를 먼저 준비합니다.

양쪽은 처리 내용을 DB에 commit한 뒤 `message.ack`를 보냅니다. ACK를 잃거나 연결이 끊기면 동일 메시지를 다시 보내고, `message_id` 중복을 제거합니다. 관제 상태는 `task_revision`이 증가할 때만 갱신하며 완료 상태를 과거 진행으로 되돌리지 않습니다. 완료 메시지는 `orders.business_status`나 고객 수령을 바꾸지 않습니다.

`receipts`에 이미 확정된 수령 근거는 `handover.receipt_recorded`로 전달합니다. 이 연결이 본인 확인·인도 승인·수령 등록 API를 대신하지 않습니다. 해당 고객 업무 API는 별도 구현 범위입니다.

검증: `./gradlew --no-daemon build`는 격리 PostgreSQL에서 예약/outbox 원자성, 실제 WebSocket 인증·저장 후 ACK, 재전송·순서 역전·고객 수령 분리를 검사합니다.

현재 서비스 DB의 `receipts.order_id UNIQUE`는 주문당 수령 1회를 뜻합니다. `receipt_packages`의 정확한 목록을 WSS `package_ids`로 전달하므로 다른 포장품의 복귀를 함께 승인하지 않습니다. 여러 차례 나누어 인도하는 경우의 고객 업무 API·DB 변경은 별도 결정이 필요하며, 이번 연결에서 기존 제약을 임의로 제거하지 않았습니다.
