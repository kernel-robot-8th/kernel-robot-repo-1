# 프로젝트 문서 안내

Confluence의 상세 문서를 찾아볼 수 있는 안내입니다. 원문 전체를 복제하지 않고 링크를 유지합니다.
문서가 존재하거나 Jira 작업이 완료되었다는 사실만으로 구현·실로봇 검증을 완료한 것으로 판단하지 않습니다.

## 요구사항과 시나리오

- [User Requirements](https://fp-team1.atlassian.net/wiki/spaces/MFS/pages/2260995): 고객·직원·관리자가 얻어야 할 결과
- [System Requirements](https://fp-team1.atlassian.net/wiki/spaces/MFS/pages/2293766): 기능별 요구사항과 우선순위
- [Scenario](https://fp-team1.atlassian.net/wiki/spaces/MFS/pages/3408141): 전체 서비스 시나리오
- [SD-01 모바일 예약 → 반출·운송](https://fp-team1.atlassian.net/wiki/spaces/MFS/pages/8060929)
- [SD-04 게이트 동행·쇼핑 모드·탑승 알림](https://fp-team1.atlassian.net/wiki/spaces/MFS/pages/7110665)

## 구조와 인터페이스

- [System Architecture](https://fp-team1.atlassian.net/wiki/spaces/MFS/pages/2555942)
- [HW Architecture](https://fp-team1.atlassian.net/wiki/spaces/MFS/pages/2752514)
- [SW Architecture](https://fp-team1.atlassian.net/wiki/spaces/MFS/pages/2261010)
- [OMX HW Spec](https://fp-team1.atlassian.net/wiki/spaces/MFS/pages/8519691)
- [Interface Specification](https://fp-team1.atlassian.net/wiki/spaces/MFS/pages/2752522)

- [ArUco 마커 ID 표와 인쇄 파일](aruco/README.md): 위치·게이트·선반·포장물품·트레이 마커 (FFT1-74)

2026-10-24 확인 시 Interface Specification 본문은 비어 있습니다.
구현 시 API·ROS 토픽·메시지 필드·단위·오류·타임아웃을 양쪽 모듈 담당자가 확인해 기록해야 합니다.

## 학습과 검증

- [OMX 실습 학습 정리 · Sprint 1](https://fp-team1.atlassian.net/wiki/spaces/MFS/pages/3506596)
- [기술 검증](https://fp-team1.atlassian.net/wiki/spaces/MFS/pages/2850831)
- [Validation](https://fp-team1.atlassian.net/wiki/spaces/MFS/pages/2555950)
- [기능별 Tech Spec 템플릿 · FFT1-38](https://fp-team1.atlassian.net/browse/FFT1-38)

학습 자료의 예제·권장값과 팀의 실제 측정 결과를 구분합니다.
검증 결과에는 대상 커밋·환경·절차·결과·로그 또는 영상 링크를 남깁니다.
로봇 작업은 빌드·시뮬레이션·실로봇 검증을 나누고, 사용 설정과 캘리브레이션 식별자를 기록합니다.

## 협업과 문서 갱신

- [Jira 프로젝트](https://fp-team1.atlassian.net/jira/projects/FFT1): 담당자·완료 조건·진행 상태
- [2026-10-21 역할 분담](https://fp-team1.atlassian.net/wiki/spaces/MFS/pages/6881282)
- [스크럼 운영 가이드 · Sprint 2](https://fp-team1.atlassian.net/wiki/spaces/MFS/pages/3506572)
- [Git-flow·PR 운영 규칙 초안](https://github.com/kernel-robot-8th/kernel-robot-repo-1/issues/1)

설계 변경은 Confluence 원문에 결정 내용을 기록하고 관련 Jira와 PR을 연결합니다.
코드가 추가되면 해당 모듈에 설치·설정·실행·검증 방법을 함께 작성합니다.
팀원 GitHub 계정을 확인한 뒤 프로필 링크와 리뷰 요청에 사용합니다.
`CODEOWNERS`를 도입할 때는 계정·저장소 권한과 담당 경로를 먼저 합의합니다.

## 구현 전에 확인할 항목

- SR 문서에 과거 요구사항과 서로 다른 OMX 대수·역할 설명이 함께 있어 최신 적용 범위를 정리해야 합니다.
- SD-01은 검토용 초안이며, SD-02·SD-03의 단계 구분도 후속 확인이 필요합니다.
- 도킹 허용 오차, 물품 안착·팔 안전 복귀 판정, 안전 정지·재개 조건은 실제 장비와 캘리브레이션을 기준으로 확인합니다.
- API·메시지 계약과 통신 실패·중복 명령 처리 규칙은 인터페이스 명세에 기록합니다.

확인되지 않은 항목을 README나 코드에서 확정 사양으로 취급하지 않습니다.
