# 디렉토리·기술 스택·CI 안내

RO:PORT는 `kernel-robot-8th/kernel-robot-repo-1` 하나에서 개발하는 monorepo입니다.
구현 디렉터리는 루트 `services/` 아래에 두고 실행 환경과 의존성 잠금 파일을 독립적으로 관리합니다.

## 디렉토리

- [roport-pinky](../services/roport-pinky/README.md): ROS 2 Jazzy·Python 3.12, Pinky 주행·도킹 실행부
- [roport-omx](../services/roport-omx/README.md): Python 코어, OMX 제어·학습 연동 준비
- [roport-control-server](../services/roport-control-server/README.md): FastAPI·Three.js 웹 UI, OFFLINE/DEMO 관제와 MQTT/UDP 모듈
- [roport-ai-server](../services/roport-ai-server/README.md): FastAPI, AI 추론 서비스 준비
- [roport-service-server](../services/roport-service-server/README.md): Java 21·Spring Boot, 서비스 업무 API 준비
- [roport-web](../services/roport-web/README.md): React·TypeScript·Vite, 고객·어드민 웹 앱

이는 현재 초기 코드의 구성입니다. 상세 스택과 OMX·LeRobot 버전은
[Issue #2](https://github.com/kernel-robot-8th/kernel-robot-repo-1/issues/2)에서 논의하며 아직 최종 확정하지 않았습니다.
통합관제 웹 전환의 의존성과 현재 범위는 해당 모듈 README/lockfile을 따릅니다. DB·실장비 연동은 후속입니다.

## CI

워크플로는 루트 `.github/workflows/`에서 관리합니다.
각 구성요소 CI는 해당 디렉토리 또는 해당 워크플로 파일 변경 시 실행합니다.
모든 CI는 수동 실행도 지원합니다.

- `pinky-ci.yml`: Jazzy 컨테이너 colcon 빌드·설치 패키지 import
- `omx-ci.yml`: uv lock 검증·Ruff·import·Python 패키지 빌드
- `control-server-ci.yml`: uv lock·Ruff·API/통신/지도 테스트·Chromium 웹 UI·패키지/자산 검증
- `ai-server-ci.yml`: uv lock 검증·Ruff·준비 상태 테스트·패키지 빌드
- `service-server-ci.yml`: Gradle 테스트·JAR 빌드·artifact 저장
- `web-ci.yml`: 두 웹 앱 타입 검사·빌드·artifact 저장
- `e2e-ci.yml`: PostgreSQL 16·서비스 서버(개발 전용 인증)·카트 패드를 띄워 Chromium으로 예약번호 확인 흐름 검사
- `labeler.yml`: PR 변경 경로에 따라 구성요소 라벨 자동 적용. 각 라벨은 고유 색상으로 관리합니다.
- `ci.yml`: 추적 중인 Markdown의 로컬 링크·공백 검사

GitHub Actions의 작업 디렉토리, 의존성 캐시 키와 artifact 경로도 각 디렉토리를 기준으로 설정합니다.
CI 성공은 로봇·GPU·실제 DB 통합 검증을 의미하지 않습니다.
배포 환경 미정으로 CD는 설정하지 않았습니다.

## 이력과 문서

분리했던 레포들의 main 이력은 subtree import로 보존했습니다.
임시 Jira 연동 검증 브랜치는 제품 코드에 병합하지 않았습니다.
요구사항·설계는 [문서 안내](README.md), 상세 스택 논의는 Issue #2를 따릅니다.
