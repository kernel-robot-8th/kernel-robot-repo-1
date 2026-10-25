# RO:PORT 작업 규칙

- 이 저장소는 단일 모노레포다. 구현 영역은 `roport-pinky`, `roport-omx`, `roport-control-server`, `roport-ai-server`, `roport-service-server`, `roport-web`이다.
- 작업 경로의 하위 `AGENTS.md`를 함께 읽는다. `CLAUDE.md`는 같은 디렉토리의 `AGENTS.md`를 가리키는 상대 symlink로 유지하고 내용을 복제하지 않는다.
- 한국어로 결론부터 간결하게 답한다. 기존 코드·호출 흐름·설정부터 확인하고 필요한 최소 변경만 한다.
- 별도 저장소·submodule·새 의존성·추상화는 요청 범위를 충족하는 데 필요한 경우에만 추가한다. 설명은 기존 README에 짧게 남기고 별도 문서는 요청받았거나 꼭 필요한 경우에만 만든다.
- 기술 스택과 실행 명령은 각 모듈의 manifest·lockfile 및 `.github/workflows/`를 기준으로 확인한다. 제안한 설계를 구현된 기능처럼 기록하지 않는다.
- 원본 오픈소스는 LICENSE와 출처·태그·SHA를 보존한다. 버전 갱신 시 정식 릴리스 태그를 확인하고 소스·UPSTREAM 기록·관련 검사를 함께 갱신한다.

## Git 및 검증

- **Git 작업 전에 [.agents/rules/git-flow.md](.agents/rules/git-flow.md)를 반드시 읽는다.** 이 저장소에서는 `feature/{이름 또는 계정}/{작업내용}`, `task/{큰 작업내용}`, `release/YYYYMMDD`를 사용하며 도구의 기본 `codex/*` 접두사를 사용하지 않는다.
- 분기 기준과 PR 대상은 Issue #1의 Git-flow를 따른다. 필요한 `develop`/`task`/`release`가 없으면 임의로 main에서 새 작업을 시작하지 말고 초기 구성을 확인한다.
- 커밋과 PR 제목: `[FFT1-번호] type: 제목`. 실제 Jira 티켓을 사용한다. 허용 type은 루트 `scripts/check-commit-message.mjs`가 기준이다.
- Node.js 24에서 루트 `npm ci`로 Husky를 설치한다. 커밋 검사 테스트는 `npm run test:commit`이다.
- main/develop/task/release에는 구현 커밋을 직접 push하지 않는다. 사용자 지시 없이 병합하지 않으며, 관련 없는 변경을 기존 PR에 섞거나 공유 이력을 임의로 다시 쓰지 않는다.
- 변경한 모듈의 CI 명령을 실행한다. 성공·실패와 미검증 항목을 구분하고 원격 push/CI 결과를 확인한 뒤 완료를 알린다.
- 빌드·단위 테스트·시뮬레이션 성공을 실로봇 검증 완료로 표현하지 않는다.

## 로봇 및 운영

- 단위·좌표계·관절 방향·캘리브레이션과 명령 소유권을 확인한다. 로봇 제어 경로와 서비스·관제·AI의 책임을 임의로 바꾸지 않는다.
- 장비 연결 확인 목적으로 bringup·connect를 호출하지 않는다. 토크 활성화·캘리브레이션 쓰기·모터 구동이 발생할 수 있다.
- 모터 구동·펌웨어·sudo·사용자 데이터 삭제 등 위험한 작업은 영향과 복구 방법을 먼저 설명하고, 장비 동작은 현장 안전 조건과 사용자 지시를 확인한다.
- 비밀값·토큰·개인 환경 파일·모델 가중치·학습 데이터·빌드 산출물을 커밋하지 않는다.
