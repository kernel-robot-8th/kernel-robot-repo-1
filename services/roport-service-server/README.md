# roport-service-server

RO:PORT · GATE CREW — 회원·주문·예약·수령 처리 API

## 기술 스택

Java 21 / Spring Boot 4.1.2 / Gradle(wrapper). 업무 데이터는 PostgreSQL(JDBC·Flyway)입니다.

## 현재 범위

고객 웹 첫 슬라이스: 세션 로그인, 주문·수령 위치·항공편 조회, 예약 가능 시간, 수령 예약 생성(`requestId` 멱등)과 조회(`/api/**`, 오류는 ProblemDetail + `code`).
업무 code가 없는 오류는 상태별 일반 code를 씁니다: 404 `NOT_FOUND`, 405 `METHOD_NOT_ALLOWED`, 406 `NOT_ACCEPTABLE`, 409 `CONFLICT`, 415 `UNSUPPORTED_MEDIA_TYPE`, 그 밖의 4xx `BAD_REQUEST`, 5xx `INTERNAL_ERROR`(내부 정보는 응답에 넣지 않고 서버 로그에만 남깁니다).
예약은 `REQUESTED`(예약 접수됨 – 현장 배정 전)까지만 만듭니다. 카트 확보·관제 수락·수령 완료가 아닙니다. 관제 WSS 연동은 아직 없습니다.

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
