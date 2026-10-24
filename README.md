# RO:PORT (로포트)

**공항 면세품 물류와 고객 안내를 연결하는 로봇 서비스 프로젝트**

커널아카데미 로보틱스 심화 캠프 주말 야간반 1기 1팀 **GATE CREW (게이트크루)**의 개발 저장소입니다.
RO:PORT는 Robot + Airport를 뜻합니다.

## 프로젝트 목표

OMX 로봇팔과 Pinky 이동로봇을 연동해 면세품 보관·운송·인도를 지원하고,
고객에게 길 안내·짐 운반·탑승시간 알림을 제공하는 시스템을 개발합니다.

- 고객: 모바일 웹에서 면세품 픽업을 예약하고 진행 상태를 확인합니다.
- 직원: 주문과 보관 위치를 조회하고, 고객 확인 후 실제 수령 완료를 기록합니다.
- 관리자: 로봇 위치·작업·오류를 확인하고 작업 지시·정지·복구를 관리합니다.

위 내용은 개발 목표입니다. 개별 기능의 구현·실로봇 검증 완료를 의미하지 않습니다.

## 시스템 구성

- **OMX**: 물품 집기·배치, 상하차 및 협동 작업을 담당합니다.
- **Pinky**: 물품 운송, 도킹, 고객 안내·동행을 담당합니다.
- **서비스 서버·Web GUI**: 고객·주문·예약 정보와 서비스 진행 상태를 관리합니다.
- **통합 관제·로봇 연동부**: 작업 배정, 로봇 명령·상태 전달, 실행 결과와 오류를 관리합니다.
- **운영 GUI**: 직원의 주문 처리와 관리자의 로봇 관제를 지원합니다.

로봇팔 작업 중에는 Pinky가 정차를 유지하고, 작업 완료와 팔 안전 복귀를 확인한 뒤 출발하도록 설계합니다.
운송 도착과 고객의 실제 수령 완료는 별도 상태로 관리합니다.
장비 대수·배치, 상세 인터페이스와 안전 판정 기준은 관련 설계 문서의 최신 합의 내용을 따릅니다.

## 현재 진행 상태

2026-10-24 확인 기준입니다. 최신 작업 상태는 [Jira](https://fp-team1.atlassian.net/jira/projects/FFT1)에서 관리합니다.

- 프로젝트 주제·사용자 요구사항 초안·역할 분담·OMX 학습 정리와 일부 Pinky 시나리오 작성이 완료되었습니다.
- System Requirements와 HW Architecture는 작성 중이며, SW Architecture는 리뷰 중입니다.
- 구성요소별 초기 코드·실행 안내·CI가 등록되어 있습니다. 업무 기능·실로봇 연동은 아직 구현·검증 전입니다.
- 시나리오와 설계 문서의 작성 상태를 기능 구현·통합 검증 상태와 구분합니다.

## 팀

[2026-10-21 역할 분담 회의록](https://fp-team1.atlassian.net/wiki/spaces/MFS/pages/6881282)을 기준으로 정리했습니다.
이후 역할 변경은 팀 합의와 원문을 확인해 반영합니다.

- 백인엽: 통합 관제 / Pinky 관제 지원
- 정승훈: OMX 개발 / 서비스 서버 연동 지원
- 이민지: Pinky 주행 / 통합 관제 지원
- 예상진: OMX 개발 / OMX 관제 지원
- 강지원: Pinky 주행 / Pinky 관제 지원
- 김도연: 서비스 시스템 구현 / Web GUI 제작 지원
- 구태경: OMX 개발 / AI 서버 구축 지원

팀원별 GitHub 계정 링크는 이름과 계정의 대응을 확인한 뒤 추가합니다.

## 저장소 구조

```text
kernel-robot-repo-1/
├── roport-pinky/
├── roport-omx/
├── roport-control-server/
├── roport-ai-server/
├── roport-service-server/
├── roport-web/
│   └── apps/
│       ├── service/
│       └── admin/
├── docs/
└── .github/workflows/
```

하나의 저장소에서 관리하고 각 디렉토리의 환경·빌드·CI는 분리합니다.
상세 기술 스택은 [Issue #2](https://github.com/kernel-robot-8th/kernel-robot-repo-1/issues/2)에서 검토 중입니다.

## 문서와 협업

- [디렉토리·기술 스택·CI 안내](docs/repositories.md)
- [설계·요구사항·검증 문서 안내](docs/README.md)
- [Jira 작업 목록](https://fp-team1.atlassian.net/jira/projects/FFT1)
- [Git-flow 및 PR 운영 규칙 논의](https://github.com/kernel-robot-8th/kernel-robot-repo-1/issues/1): 현재 합의 전 초안입니다.

요구사항·상세 설계·회의 결정은 Confluence, 작업 담당과 진행 상태는 Jira,
코드·실행 방법·코드 변경 이력은 GitHub에서 관리합니다.
PR에는 관련 Jira 키와 검증 결과를 기록합니다.
Confluence와 Jira 링크는 해당 워크스페이스 접근 권한이 필요할 수 있습니다.
