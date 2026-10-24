# roport-service-server

RO:PORT · GATE CREW — 회원·주문·예약·수령 처리 API

## 기술 스택

Java 21 / Spring Boot 4.1.2 / Maven. 업무 데이터는 PostgreSQL 기준이며 DB 연결·마이그레이션·인증은 후속 구현입니다.

## 현재 범위

개발 시작용 구조와 CI를 제공합니다. 로봇 제어·업무 기능·모델 추론은 아직 구현하지 않았습니다.
CI 성공은 실로봇·GPU·통합 검증 완료를 의미하지 않습니다.
배포 환경 미정으로 CD와 자동 배포는 설정하지 않았습니다.

## 관련 문서

- [프로젝트 개요](https://github.com/kernel-robot-8th/kernel-robot-repo-1)
- [요구사항·설계 문서](https://github.com/kernel-robot-8th/kernel-robot-repo-1/blob/main/docs/README.md)
- [Jira](https://fp-team1.atlassian.net/jira/projects/FFT1)

PR에 관련 Jira 키, 변경 내용, 검증 결과와 미검증 항목을 기록합니다.
실로봇 적용 전 장비·설정·캘리브레이션, 안전 정지·복구 절차를 확인합니다.

## 개발 방법

Java 21과 Maven 3.9 이상을 사용합니다.

```bash
mvn verify
mvn spring-boot:run
```

상태 확인: `http://127.0.0.1:8080/actuator/health`. 현재 업무 API·인증·DB는 없습니다.
