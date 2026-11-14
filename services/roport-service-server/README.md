# roport-service-server

RO:PORT · GATE CREW — 회원·주문·예약·수령 처리 API

## 기술 스택

Java 21 / Spring Boot 4.1.2 / Gradle(wrapper). 업무 데이터는 PostgreSQL(JDBC·Flyway)입니다.

## 현재 범위

고객 웹 첫 슬라이스: 세션 로그인, 주문·수령 위치·항공편 조회, 예약 가능 시간, 수령 예약 생성(요청 ID 없음, 주문당 활성 예약 1개 — 두 번째 요청·재전송은 409 `ACTIVE_PICKUP_EXISTS`)과 조회(`/api/**`, 오류는 ProblemDetail + `code`).
업무 code가 없는 오류는 상태별 일반 code를 씁니다: 404 `NOT_FOUND`, 405 `METHOD_NOT_ALLOWED`, 406 `NOT_ACCEPTABLE`, 409 `CONFLICT`, 415 `UNSUPPORTED_MEDIA_TYPE`, 그 밖의 4xx `BAD_REQUEST`, 5xx `INTERNAL_ERROR`(내부 정보는 응답에 넣지 않고 서버 로그에만 남깁니다).
예약은 `REQUESTED`(예약 접수됨 – 현장 배정 전)까지만 만듭니다. 카트 확보·관제 수락·수령 완료가 아닙니다. 관제 WSS 연동은 아직 없습니다.

예약 QR·카트 확인(FFT1-178): `POST /api/pickups/{pickupId}/qr-token`은 본인 활성 예약에 새 QR 토큰을 발급하고 이전 토큰을 폐기합니다(원문은 응답에만, DB에는 SHA-256 해시만). 만료는 발급 후 `service.qr.ttl`(기본 10분)이며 응답의 `expiresAt`으로 알려 줍니다.
`POST /cart-api/verifications`는 QR 토큰 또는 예약번호로 예약을 찾아 `verifications`에 결과를 기록합니다(새 확인 201, 같은 `verificationRequestId` 재전송 200, 다른 내용 409 `IDEMPOTENCY_CONFLICT`, 예약 없음 404 `RESERVATION_NOT_FOUND`). 이미 쓴 요청 ID에 없는 값을 보내면 404보다 409가 우선합니다.
카트별로 확인 실패(예약 없음·거절)가 `service.verify.failure-window`(기본 1분) 안에 `service.verify.max-failures`(기본 5회)에 이르면 새 확인 요청은 429 `TOO_MANY_ATTEMPTS`입니다(FFT1-182). 기록은 서버 메모리에만 두며, 카트 ID를 위조할 수 있는 동안에는 완전한 방어가 아닙니다.
**확인은 인도 승인이 아닙니다.** 카트 배정 대조(`cartAssignmentChecked=false`)·인도 승인·수령 기록은 관제 연동 후입니다.
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
