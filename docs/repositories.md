# 개발 레포·기술 스택·CI

2026-10-24 개발 시작 기준입니다. 각 레포는 별도 Git 저장소이며, 이 저장소는 전체 프로젝트와 문서 연결을 관리합니다.

## 개발 레포

- [roport-pinky](https://github.com/kernel-robot-8th/roport-pinky): Ubuntu 24.04, ROS 2 Jazzy, Python 3.12, colcon. 주행·도킹·현장 명령 실행을 담당합니다.
- [roport-omx](https://github.com/kernel-robot-8th/roport-omx): Python 3.12 코어, uv, Ruff. OMX 제어·협동 동작·시연 수집·학습을 담당합니다. LeRobot·PyTorch는 장비 드라이버와 CUDA 호환성을 확인한 별도 환경에서 버전을 고정합니다.
- [roport-control-server](https://github.com/kernel-robot-8th/roport-control-server): Python 3.12, FastAPI, Uvicorn. 작업 배정·상태 전이·자원 점유·명령 이력과 복구를 담당합니다. PostgreSQL을 저장소 기준으로 잡되 아직 연결하지 않았습니다. ROS 2 Jazzy Bridge는 API와 별도 프로세스로 실행합니다.
- [roport-ai-server](https://github.com/kernel-robot-8th/roport-ai-server): Python 3.12, FastAPI, Uvicorn. 모델 로딩·추론을 담당합니다. PyTorch·GPU 런타임은 모델과 배포 환경 확정 후 추가합니다. 실제 모델이 없으므로 `/readyz`는 HTTP 503을 반환합니다.
- [roport-service-server](https://github.com/kernel-robot-8th/roport-service-server): Java 21, Spring Boot 4.1.2, Maven. 회원·주문·예약·수령 업무를 담당합니다. PostgreSQL을 DB 기준으로 잡되 DB 연결·인증·업무 API는 후속 구현입니다.
- [roport-web](https://github.com/kernel-robot-8th/roport-web): Node.js 24, React 19, TypeScript 7, Vite 8, npm workspaces. `apps/service`와 `apps/admin`을 별도로 빌드합니다.

## 책임 경계

서비스 서버는 고객·주문·예약·수령 업무 데이터를 소유합니다.
관제 서버는 로봇 작업 진행, 배정, 타임아웃·복구와 실행 결과를 소유합니다.
ROS 2 Bridge는 명령·결과를 전달하며 업무 결정을 대신하지 않습니다.
AI 서버는 추론 결과를 반환하고 로봇 모터를 직접 제어하지 않습니다.
OMX의 정책 추론을 원격화할지는 제어 주기·통신 지연·연결 끊김 시 동작을 검증한 뒤 정합니다.

API·ROS 메시지 이름과 데이터 스키마는 Confluence 인터페이스 명세에서 양쪽 담당자가 합의합니다.
기능은 아직 구현 전이며, 위 구조가 통합·실장비에서 검증되었다는 뜻은 아닙니다.

## CI

각 레포의 GitHub Actions는 모든 브랜치의 `push`, `pull_request`, 수동 실행을 지원합니다.

- Pinky: ROS 2 Jazzy 컨테이너에서 colcon 빌드 및 설치 패키지·rclpy import
- OMX: 잠금 파일 기준 의존성 설치, Ruff 검사·포맷 확인, 패키지 import·배포 패키지 빌드
- 관제·AI 서버: 잠금 파일 기준 설치, Ruff, 상태 확인 API 테스트, 배포 패키지 빌드
- 서비스 서버: Java 21에서 Maven verify, 애플리케이션 기동 테스트, JAR artifact 저장
- 웹: npm ci, 두 앱 TypeScript 검사·프로덕션 빌드, 정적 파일 artifact 저장
- 통합 레포: 로컬 Markdown 링크 및 공백 검사

로봇·GPU·실제 DB를 사용하는 테스트는 아직 없습니다.
CI는 기본 구조와 빌드 가능성을 확인하며 시연 성공을 판정하지 않습니다.
CI가 실행된다는 사실과 병합 전 성공을 강제하는 branch protection은 별개입니다. 보호 규칙은 아직 설정하지 않았습니다.

## CD

배포 환경 미정으로 CD·운영 배포는 구성하지 않았습니다.
배포 PC/서버, CPU·GPU, 네트워크, 도메인, 비밀정보 저장 방법과 복구 절차를 정한 뒤 추가합니다.
실로봇 자동 배포·모터 구동은 CI에 포함하지 않습니다.

## 버전과 공식 자료

Python 레포의 `uv.lock`, 웹의 `package-lock.json`, 서비스 서버의 Spring Boot parent 버전으로 빌드 의존성을 관리합니다.
최신 버전을 무조건 적용하지 않고 장비·런타임 호환성을 확인해 갱신합니다.

- [ROS 2 Jazzy 설치](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [LeRobot 설치](https://huggingface.co/docs/lerobot/installation)
- [Spring Boot 요구 환경](https://docs.spring.io/spring-boot/system-requirements.html)
- [Vite 시작하기](https://vite.dev/guide/)
- [uv와 GitHub Actions](https://docs.astral.sh/uv/guides/integration/github/)
