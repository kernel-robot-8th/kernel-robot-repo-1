# 상세 기술 스택과 호환성 기준

작성 기준: 2026-10-24. 이 문서는 현재 레포 설정과 후속 개발 기준을 구분합니다.
`현재`는 코드·잠금 파일·CI에서 확인한 내용이며, `개발 기준`은 앞으로 적용할 설계입니다.
현재 CI 성공 범위는 초기 패키지·상태 확인 API·기본 웹 화면이며 실제 업무·로봇 통합은 포함하지 않습니다.

## 1. 공통 환경과 버전 관리

- 개발 호스트 기준: Ubuntu 24.04 LTS. Pinky 보드의 실제 OS·CPU 아키텍처는 별도 확인합니다.
- ROS 2: Jazzy. ROS 패키지는 apt 버전 목록 또는 소스 커밋을 검증 기록에 남깁니다.
- 일반 Python API·코어: Python 3.12, uv 0.13.5, 레포별 `.venv`와 `uv.lock`.
- OMX 학습·실행: [별도 호환성 문서](omx-compatibility.md)의 LeRobot 환경을 사용합니다. ROS Python 환경과 패키지를 공유하지 않습니다.
- 웹: Node.js 24 계열, npm, `package-lock.json`. 팀 런타임의 패치 버전은 배포 이미지 선택 시 고정합니다.
- Java: JDK 21, Maven 3.9 계열. Spring 관리 라이브러리는 Boot BOM에 맡깁니다.
- PostgreSQL 개발 기준: 17.11. 서비스 업무 DB와 관제 운영 DB는 DB·계정으로 분리하고 직접 교차 수정하지 않습니다. 아직 레포에 DB를 연결하지 않았습니다.
- 배포 환경이 정해지면 컨테이너 태그·digest, CPU 아키텍처, GPU 드라이버와 CUDA wheel 변형을 함께 기록합니다. `latest`로 실장비 환경을 재현하지 않습니다.

## 2. roport-pinky

### 런타임·라이브러리

- 현재: ROS 2 Jazzy, Python 3.12, `ament_python`, colcon, `rclpy` 패키지 기반.
- 개발 기준: Nav2 Jazzy의 경로 실행·도착/취소/실패 응답을 현장 Command Executor와 연결합니다.
- 지도·위치 추정: 지도 서버와 AMCL을 기준으로 하되, Nav2 주행 채택과 실제 센서·TF·odometry를 먼저 검증합니다. 지도 작성 도구는 초기 주행 범위 확정 후 추가합니다.
- 좁은 도킹의 정렬 방식·마커·카메라·LiDAR 사용 여부는 장비 시험으로 정합니다. Nav2 목표 도착만으로 상하차 가능한 정밀 도킹을 판정하지 않습니다.
- 저장: 지도·파라미터는 Git에서 관리하고 rosbag2·영상은 별도 artifact 저장소에 둡니다.

### 인터페이스·안전

- Topic: 위치·연결·장비 상태. Action: 이동·도킹 등 장시간 작업. Service: 짧은 조회·설정 요청.
- ROS 관례 기준: 길이 m, 각도 rad, 시간 s. `map → odom → base_link` 관계와 센서 프레임은 실제 장비 기준으로 검증합니다.
- 좌표에는 frame ID와 시각을 포함합니다. 연결 끊김 시 마지막 위치를 현재 위치로 표시하지 않습니다.
- 속도·가속도·정지 거리·도킹 허용 오차는 숫자를 임의로 정하지 않고 실측 후 설정 파일로 고정합니다.
- 상하차 동안 정차 유지, 화물 상태·팔 안전 복귀·출발 허가를 모두 확인합니다. 통신 상실 시 현장 정지 기준을 적용합니다.

### CI·검증 확장

- 현재: Jazzy 컨테이너에서 colcon build, 설치 패키지와 rclpy import.
- 다음: 상태 전이·중복 명령·취소 테스트, TF·인터페이스 검사, Gazebo Harmonic에서 경로·실패 시나리오 검증.
- 실로봇: 도킹 반복 오차, 교행·장애물·연결 끊김·화물 탑재 조건을 따로 기록합니다.

## 3. roport-omx

- 현재: Python 3.12 빈 코어 패키지, uv 0.13.5, Ruff 0.17.2. LeRobot·PyTorch는 아직 설치 의존성에 포함하지 않았습니다.
- 개발 기준: 기존 강의 재현과 데이터·체크포인트 확인은 LeRobot v0.4.4를 우선 기준으로 삼습니다. 학습·실행용 Python 3.10 환경을 코어 CI와 분리합니다. 실제 사용 버전을 받은 뒤 확정합니다.
- 신규 LeRobot 0.6.2 계열은 업그레이드 후보이며, 검증 없이 구버전 환경을 교체하지 않습니다.
- 정책 기준: 제한된 pick-and-place 작업의 ACT부터 시작합니다. 양팔 협동은 단일 팔 정책 두 개를 동시에 실행하는 것만으로 구현 완료로 보지 않습니다.
- 하드웨어 접근은 OMX Executor 하나가 소유합니다. ROS/MoveIt 드라이버와 LeRobot이 같은 USB 버스를 동시에 제어하지 않습니다.
- 자세한 버전·단위·데이터·학습·동기화 기준은 [OMX 호환성 문서](omx-compatibility.md)를 따릅니다.

## 4. roport-control-server

### 런타임·저장

- 현재 고정 버전: Python 3.12, FastAPI 0.162.0, Uvicorn 0.54.0, Pydantic 2.14.1, pytest 9.1.1, Ruff 0.17.2. 전이 의존성은 `uv.lock` 기준입니다.
- 개발 기준: PostgreSQL 17.11, SQLAlchemy 2.0 계열 AsyncSession, Psycopg 3 async 드라이버, Alembic. 실제 DB 구현 PR에서 패치 버전과 lock을 함께 고정합니다.
- API, 작업 실행기, ROS 2 Bridge를 별도 프로세스로 실행합니다. 같은 레포에서 관리하되 ROS 패키지와 API 가상환경을 분리합니다.
- 초기에는 작업 실행기 한 인스턴스만 사용합니다. API worker를 늘린다고 작업 실행기를 중복 기동하지 않습니다.
- 로봇 최신 telemetry는 메모리에서 관리하고, 작업·명령·자원 점유·실패·복구 이력은 DB에 저장합니다. Redis·메시지 브로커는 초기 필수 의존성이 아닙니다.

### 통신·상태 소유권

- 서비스 서버와는 HTTP/JSON API로 작업 요청·조회부터 구현합니다. 첫 버전은 상태 조회 방식이며 필요할 때 이벤트 전달을 추가합니다.
- 관제와 ROS Bridge 사이의 기존 TCP 경계를 유지합니다. 구현 전 프레임 길이·메시지 형식·연결 복구·메시지 크기 제한을 인터페이스 명세에서 정합니다. TCP read 한 번을 메시지 하나로 취급하지 않습니다.
- Bridge는 rclpy로 로봇 Action·Topic·Service를 연결하며 작업 성공·실패를 독자적으로 결정하지 않습니다.
- 작업 ID와 command ID, attempt, 대상 로봇·물품, 상태 버전과 시각을 구분합니다. 전송 재시도는 같은 command ID를 유지하고, 새 실행 시도는 정책에 따라 별도로 생성합니다.
- 명령 접수·실행·물리적 완료를 별도 상태로 관리합니다. 응답 유실 후 실제 완료 여부를 조회하고 물리 작업을 무조건 재전송하지 않습니다.

### 다음 CI

PostgreSQL service container 기반 마이그레이션·트랜잭션 테스트, 배정 경합·중복 지시·재시작 복구 테스트,
가짜 Bridge로 응답 유실·순서 뒤바뀜·연결 끊김을 검증합니다. 이 테스트들은 아직 구현 전입니다.

## 5. roport-ai-server

- 현재: 관제와 같은 Python·FastAPI 기반. `/healthz`는 생존 확인이며 모델 미설정 상태의 `/readyz`는 503입니다.
- 개발 기준: API와 GPU 추론 worker를 분리합니다. CPU/GPU 추론을 async API 이벤트 루프에서 직접 실행하지 않습니다.
- 작업 모델은 OMX 담당자의 정책·관측 계약에 맞춥니다. ACT 추론을 HTTP 요청 한 번당 모터 명령 한 번으로 연결하지 않습니다.
- OMX 정책을 원격 실행할 경우 같은 LeRobot commit·PyTorch 계열·processor·통계 파일을 사용합니다. 다른 비전 모델은 별도 worker 환경으로 분리할 수 있습니다.
- GPU worker 하나당 초기 모델 인스턴스 하나, 제한된 큐·동시 실행 수·타임아웃을 설정합니다. 큐가 가득 찼을 때 거절·대기를 명시합니다.
- 모델 식별: 모델 revision 또는 artifact hash, 정책 설정, 전처리기, 데이터 통계, 입력 feature·단위·순서를 함께 기록합니다.
- 영상 전송 시 해상도·채널 순서·압축·촬영 시각과 지연을 계약에 넣습니다. 지연된 관측의 행동을 현장에서 그대로 실행하지 않습니다.
- CPU CI는 모델 로딩 실패·입력 형식·준비 상태·타임아웃을 검증합니다. GPU smoke test와 추론 지연·VRAM 측정은 장비 확보 후 추가합니다.
- GPU·드라이버·모델이 미확인이라 CUDA·TensorRT·ONNX Runtime은 아직 채택하지 않습니다. 먼저 원본 PyTorch 경로의 결과를 기준으로 삼습니다.

## 6. roport-service-server

- 현재: Java 21, Spring Boot 4.1.2, Spring MVC, Actuator, Maven. Boot BOM이 Spring·Tomcat·테스트 의존성을 관리합니다.
- 개발 기준: PostgreSQL 17.11, Spring Data JPA, Flyway, Bean Validation, Spring Security. DB·인증 구현 시 필요한 starter만 추가합니다.
- 업무 소유권: 회원, 주문, 포장품, 예약, 수령 확인. 로봇 위치·배정·명령 실행 상태는 관제의 소유이며 조회용 사본만 유지합니다.
- 관제 호출: Spring RestClient와 명시적 연결·응답 타임아웃. WebFlux 전환은 초기 필수가 아닙니다.
- 예약 저장과 운송 요청 사이의 실패를 다루기 위해 DB에 전송 대기 상태 또는 outbox를 함께 기록하고, idempotency key로 재시도합니다. API 요청 동안 로봇 완료를 기다리지 않습니다.
- 웹 인증 초기 기준: 서버 세션 + HttpOnly 쿠키, 상태 변경 요청의 CSRF 보호, 고객·직원·관리자 권한 검사. 운영에서는 HTTPS와 실제 배포 도메인을 확정해 쿠키·CORS를 설정합니다.
- 준비된 시연용 주문·항공편 데이터와 외부 실시간 연동을 구분합니다. 데이터가 더미임을 API·화면에서 명확히 합니다.
- 다음 CI: Flyway migration, PostgreSQL 통합 테스트, 예약 중복·수령 중복·권한·관제 장애 테스트. H2만으로 PostgreSQL 동작을 검증했다고 보지 않습니다.

## 7. roport-web

- 현재: Node.js 24 계열, React 19.3.1, TypeScript 7.0.2, Vite 8.3.5, npm workspaces.
- 앱 경계: `apps/service`는 고객 예약·진행 조회, `apps/admin`은 직원 주문 처리·관리자 화면입니다. 두 앱은 별도 빌드·배포할 수 있습니다.
- 개발 기준: React Router의 경로 분리, 브라우저 fetch 기반 API 모듈, React state부터 사용합니다. 서버 캐시 요구가 생기면 TanStack Query를 도입하며 초기에 Redux를 추가하지 않습니다.
- 스타일: 기본 CSS부터 구성합니다. 공통 UI 컴포넌트는 실제로 양쪽 앱에서 중복될 때 추출합니다.
- 서비스 서버 REST API를 기본 창구로 사용합니다. 운영 관제 화면에 실시간 telemetry가 필요하면 관제의 인증된 조회 전용 API/SSE와 접속 범위를 별도로 정합니다. 브라우저가 ROS DDS나 로봇 USB에 직접 접근하지 않습니다.
- 위치·주문·작업 정보를 분리하고 명령 접수와 실제 완료를 다르게 표시합니다. 상태 수신 시각·연결 끊김·오류를 화면에 노출합니다.
- 다음 CI: Vitest·Testing Library로 화면/상태 검사, Playwright로 예약→준비 상태 조회·직원 수령 확인 흐름 검사. 패키지는 기능 테스트 추가 시 lock과 함께 도입합니다.
- 브라우저에 전달되는 설정에는 비밀키를 넣지 않습니다. 어드민 앱을 별도 URL에 배치해도 권한 검사는 서버에서 합니다.

## 8. 구현 순서와 확정 조건

1. OMX 실제 설치 버전·장비·GPU·기존 데이터와 체크포인트 확인. 데이터·설정 사본을 보존하고 기존 환경을 덮어쓰지 않습니다.
2. LeRobot 고정 commit, Python·torch·torchvision·영상 decoder 조합을 별도 lock으로 재현합니다.
3. 서비스↔관제 작업 접수·조회 API와 관제↔Bridge TCP 메시지 계약을 정의합니다.
4. 실제 DB를 연결하고 중복 요청·응답 유실·재시작 복구 테스트를 CI에 추가합니다.
5. 단일 OMX 작업과 Pinky 도킹을 각각 검증한 뒤 상하차 연동을 진행합니다.
6. 단일 작업을 통과한 뒤 양팔 협동·다중 Pinky·원격 GPU 추론을 검증합니다.

현재 문서 추가만으로 계획한 라이브러리·DB·테스트가 설치되지는 않습니다.
CD는 배포 PC·도메인·접속 방식·비밀정보·복구 절차 확정 후 별도 구현합니다.

## 공식 근거

- [LeRobot v0.4.4 pyproject](https://github.com/huggingface/lerobot/blob/v0.4.4/pyproject.toml)
- [LeRobot 분석 기준 commit pyproject](https://github.com/huggingface/lerobot/blob/c1789b1306ca451cdc61c4576c3eba691bd9b974/pyproject.toml)
- [ROBOTIS OMX 설정 경로](https://docs.robotis.com/docs/systems/omx/quick_start_guide/setup_guide/)
- [Nav2 Jazzy 설치](https://docs.nav2.org/jazzy/getting_started/build_and_install/)
- [Spring Boot 요구 환경](https://docs.spring.io/spring-boot/system-requirements.html)
- [PostgreSQL 버전 정책](https://www.postgresql.org/support/versioning/)
- [SQLAlchemy async API](https://docs.sqlalchemy.org/en/20/orm/extensions/asyncio.html)
- [Vite](https://vite.dev/guide/)
