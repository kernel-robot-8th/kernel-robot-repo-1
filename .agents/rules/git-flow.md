# Git-flow 작업 규칙

기준: 이 문서와 저장소의 활성 GitHub ruleset이다. [Issue #1](https://github.com/kernel-robot-8th/kernel-robot-repo-1/issues/1)은 초기 `develop` 기반 논의 기록이며 현재 절차의 기준이 아니다.

## 작업 전 확인

- 브랜치 생성·커밋·push·PR 작업 전에 이 파일을 읽는다. 다른 도구의 기본 브랜치 접두사보다 이 저장소 규칙을 우선한다.
- 현재 브랜치·작업 트리·원격 상태와 관련 Jira를 확인한다. 작업 브랜치와 PR 대상을 먼저 정한다.
- 열린 PR의 브랜치·대상을 임의로 바꾸지 않는다. 공유 브랜치 이력을 rebase/force push로 다시 쓰지 않는다.

## 브랜치

- **`main`**: 기능 통합과 검증을 마친 기준 브랜치다. 이 단기 프로젝트에서는 `develop`을 만들거나 사용하지 않는다.
- **`feature/{이름 또는 GitHub 계정}/{작업내용}`**: 독립 기능·수정 브랜치다. 최신 `main`에서 만들고 `main`을 대상으로 PR을 보낸다.
- **`task/{큰 작업내용}`**: 여러 feature를 묶어 공동 개발·통합할 때만 쓰는 선택 브랜치다. 최신 `main`에서 만들고, 하위 feature PR을 모두 통합·검증한 뒤 `main`으로 PR을 보낸다.

브랜치 이름은 영문 소문자·숫자와 하이픈을 사용한다. feature에는 작업자 이름 또는 GitHub 계정을 넣는다.

```text
feature/jsh0116/update-build-guide
task/robot-navigation
```

작업 순서는 다음과 같다.

```text
독립 작업: main → feature → PR → main
공동 작업: main → task → feature → PR → task → 통합 검증 → PR → main
```

작업 시작 전에 관련 Issue/Jira에 작업 범위, 완료 조건, 독립 작업 또는 task 사용 여부를 기록한다. feature PR은 처음 분기한 `main` 또는 `task`를 대상으로 한다.

## PR 및 병합

- `main`과 `task/**`에는 직접 push하지 않고 PR로 반영한다. 작업 커밋은 `feature/**`에 push한다.
- 작성자 외 팀원 1명 이상의 승인을 받은 뒤 PR 작성자가 병합한다. 새 커밋이 올라오면 기존 승인은 무효화되며, 마지막 push를 하지 않은 사람이 승인해야 한다.
- `feature → main`과 `feature → task`는 **Squash and merge**를 사용한다.
- `task → main`은 통합 이력을 보존하도록 **Create a merge commit**을 사용한다.
- PR 제목은 `[FFT1-번호] type: 제목` 형식을 따른다. 실제 Jira 작업을 연결하고 `.github/pull_request_template.md`에 변경 내용과 검증 결과를 기록한다.

## 보호와 정리

- GitHub ruleset은 `main`과 `task/**`에 적용된다. PR 승인 요구와 force push 차단이 활성화되어 있다.
- PR이 병합되면 GitHub가 head 브랜치를 자동 삭제한다. 삭제된 브랜치는 GitHub에서 복원할 수 있다.
- `task`를 `main`에 병합하기 전에 그 task를 대상으로 열린 feature PR과 후속 작업을 모두 마친다. 병합 후 task 브랜치가 자동 삭제된다.

## 커밋 검사

Node.js 24에서 저장소 루트의 `npm ci`로 Husky를 설치한다. 허용 제목 형식과 type은 `scripts/check-commit-message.mjs`를 따른다. 검사 테스트는 `npm run test:commit`이다.
