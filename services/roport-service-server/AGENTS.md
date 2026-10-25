# 서비스 서버

- Java 21 / Spring Boot / Maven을 사용한다. 버전은 `pom.xml`이 기준이다.
- 앱은 `src/main/java/com/roport/service/`, 검사는 `mvn --batch-mode --no-transfer-progress verify`다.
- 기존 패키지 구조와 Spring 표준 기능을 우선한다. 예약·주문·고객 처리 변경은 API 계약·입력 검증·오류 응답과 관련 테스트를 함께 확인한다.
- 관제/로봇 연동에서 접수·진행·완료·실패를 구분한다. HTTP 응답 성공이나 로봇의 도착을 고객 인도 완료로 처리하지 않는다.
- 실서비스 데이터나 장비를 테스트에서 사용하지 않는다. DB·인증·배포 환경이 확정됐다고 가정해 의존성을 추가하지 않는다.
