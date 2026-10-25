# Git-flow 작업 규칙

기준: [GitHub Issue #1](https://github.com/kernel-robot-8th/kernel-robot-repo-1/issues/1).
사용자의 적용 지시에 따라 에이전트는 이 규칙을 따른다. GitHub 보호 설정의 적용 여부와는 별개다.

## 작업 전 확인

- 브랜치 생성·커밋·push·PR 작업 전에 이 파일을 읽는다. 다른 도구의 기본 브랜치 접두사보다 이 저장소 규칙을 우선한다.
- 현재 브랜치·작업 트리·원격 상태와 관련 Issue를 확인한다. 이번 작업의 생성 기준 브랜치, 작업 브랜치 이름, PR 대상을 먼저 명확히 한다.
- 필요한 develop/task/release가 없으면 main이나 codex/*로 임의 대체하지 않는다. 초기 브랜치 구성 또는 예외를 사용자와 확인한다.
- 열린 PR의 브랜치·대상을 임의로 바꾸지 않는다. 공유 브랜치 이력을 rebase/force push로 다시 쓰지 않는다.

## 이름과 역할

- main: 검증한 배포·시연 기준 코드.
- develop: 다음 배포 기능을 통합하는 상시 브랜치.
- feature/{이름 또는 GitHub 계정}/{작업내용}: 개별 구현·수정. 예: feature/jsh0116/add-navigation-action.
- task/{큰 작업내용}: 여러 feature의 공동 개발·통합. 예: task/robot-navigation. 모든 작업에 필수는 아니다.
- release/YYYYMMDD: 한국 시간 기준 배포·시연 예정일. 같은 날 추가 배포는 release/YYYYMMDD-2처럼 구분한다.
- 이름은 영문 소문자·숫자, 단어 구분은 하이픈을 사용한다. feature에는 실제 작업자 이름/계정을 넣는다.
- codex/*, 임의의 새 브랜치 종류, hotfix 규칙을 기본값으로 만들지 않는다. 배포 후 긴급 수정 절차는 별도 합의까지 보류다.
- Jira 번호는 브랜치 이름에 필수가 아니다. 커밋·PR 제목은 루트의 [FFT1-번호] type: 제목 규칙을 따른다.

## 분기와 PR 대상

- 초기 저장소 세팅 중 사용자가 명시한 경우에만 main → feature/{작업자}/{내용} → main을 허용한다. 일반 개발의 기본 분기/대상을 main으로 바꾸지 않는다.
- 독립 작업: develop → feature/{작업자}/{내용} → develop.
- 공동 작업: develop → task/{내용} → feature/{작업자}/{내용} → 같은 task → develop.
- 배포 준비: develop → release/YYYYMMDD → main → develop 동기화.
- release 검증 중 수정은 해당 release에서 feature를 만들고 같은 release로 PR을 보낸다. 신규 기능은 다음 배포로 넘긴다.
- feature의 PR 대상은 처음 분기한 task/develop/release다. 임의로 main을 대상으로 잡지 않는다.
- main, develop, task/**, release/**에는 구현 커밋을 직접 push하지 않는다. 구현 변경은 feature/**에서 PR로 반영한다.

## 리뷰·병합·동기화

- PR 하나에는 하나의 목적을 담는다. 관련 Issue/Jira, 변경 내용, 검증 결과·미검증 항목을 기록한다.
- PR을 만들거나 설명을 고칠 때 `.github/pull_request_template.md`의 항목과 순서를 따른다. Jira, 변경 내용, 검증한 내용과 미검증 이유를 채우고, 해당 없는 선택 항목만 템플릿 안내대로 뺀다.
- 작성자 외 팀원 1명 이상 승인 후 PR 작성자가 병합한다. 사용자 지시 없이 에이전트가 병합하지 않는다.
- feature → task/develop은 Squash and merge가 기본이다. task → develop과 배포·동기화 PR은 merge commit을 사용한다.
- 작업 전 최신 원격 상태를 확인하고, 진행 중 feature에는 원래 기준 task/develop/release의 변경을 반영한다.
- 장기 task에는 필요한 develop 변경을 merge한다. 충돌 해결 후 영향을 받는 검증을 다시 실행한다.
- 배포 후 main 커밋에 vYYYYMMDD.순번 태그를 남기고 main을 develop에 merge해 변경을 동기화한다.
- 완료 브랜치는 열린 PR·후속 작업이 없는지 확인한 뒤 정리한다. 기존 배포 브랜치·태그를 덮어쓰지 않는다.
- 로봇 검증은 빌드·시뮬레이션·실장비를 구분하고 사용 설정·캘리브레이션과 결과를 남긴다.
