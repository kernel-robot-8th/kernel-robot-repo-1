# roport-omx

RO:PORT · GATE CREW — OMX 제어·협동 동작·시연 수집·학습

## 기술 스택

Python 3.12 코어 패키지 / uv / Ruff. 학습·정책 실행은 LeRobot·PyTorch를 사용하되 OMX 드라이버·CUDA 호환성을 확인한 별도 환경에서 버전을 고정합니다.

## 현재 범위

팔별 분담 시나리오의 source/mock 실행 코어와 CI를 제공합니다. 실제 로봇 구동·학습된 ACT 정책·물리 파지 판정은 아직 연결하지 않았습니다.
CI 성공은 실로봇·GPU·통합 검증 완료를 의미하지 않습니다.
배포 환경 미정으로 CD와 자동 배포는 설정하지 않았습니다.

## 팔별 분담 실행 코어

`src/roport_omx/dedicated.py`의 `Profile`과 `DedicatedHost`가 입고·반출 담당 팔, 단일 포장품·슬롯, 정차 permit, 양팔 관측, 정책·보정 버전을 검사합니다. SQLite에 명령을 저장한 뒤 접수하고 같은 명령은 재실행하지 않습니다. 재시작 시 불명확한 실행은 `RECONCILING`으로 남깁니다.

`GRASPING → WAIT_GRASP → TRANSFERRING → WAIT_SEATED → RELEASING → WAIT_RELEASE → RETURNING → WAIT_SAFE → SAFE_RETURNED` 순서로 진행합니다. 단계 경계와 다음 정책 시작 시 AI reset 응답을 확인하며, 정상 대기와 오류 보류를 구분합니다. 최신 관측·확인 입력 없이 chunk 종료나 그리퍼 위치만으로 완료하지 않습니다. Host는 출발을 허가하지 않습니다.

```bash
uv sync --locked
uv run python scripts/dedicated_demo.py --mock
bash ci/test.sh
```

예제의 자세·시간 제한·물품 ID는 synthetic fixture입니다. 코어는 검사한 6개 위치 목표를 데이터로 반환할 뿐, 시리얼·토크·모터 호출을 제공하지 않습니다. `HOLD`도 소프트웨어의 후보 차단 상태이며 물리 정지를 증명하지 않습니다. 현장 실행에는 실측 프로파일, 영상·센서/감독자 확인 입력, 실제 정지·지지·수동 회수 절차와 별도 장비 어댑터가 필요합니다. 팔 5축 정규화/°와 그리퍼 0~100 단위를 섞지 마세요.

### 세 모듈 연결 예제와 개발 브랜치

통합 브랜치에서는 관제의 명령·정차 permit을 Host에 넣고, 실제 localhost UDP 소켓으로 AI fixture 후보를 받는 연속 예제를 실행할 수 있습니다.

```bash
uv run python scripts/dedicated_integration.py --mock
```

입고 완료·출발 판단·P7 이탈 뒤에 빈 Pinky3 반출을 배정합니다. `DELIVERY_READY`는 배송 준비이며 고객 수령 완료가 아닙니다. MQTT envelope는 이 예제에서 메모리상 encode/decode하고, 실제 broker 왕복은 관제의 localhost Mosquitto 검사에서 따로 확인합니다. 물체 이동·접촉·하중·학습 성능을 재현하는 시뮬레이션은 아닙니다. MuJoCo는 실제 기구·그리퍼·물품·선반의 접촉 모델을 준비한 뒤 후속 물리 검증에 검토합니다.

개발 브랜치는 `task/omx`를 기준으로 나눕니다. 현재 PR은 생성하지 않습니다.

- `feature/jsh0116/omx-dedicated-control`: 관제 명령·permit·P7 작업 순서.
- `feature/jsh0116/omx-dedicated-host`: Host 프로파일·상태기계·SQLite 이력과 단독 mock 예제.
- `feature/jsh0116/omx-dedicated-ai`: 정책 라우터·UDP adapter와 wheel 구성.
- `feature/jsh0116/omx-dedicated-integration`: 위 세 브랜치를 합친 연결 예제와 회귀 검사. 검토할 때는 각 기능 브랜치부터 확인하고 통합 브랜치에서 연결 변경을 확인합니다.

의존성: 통합 예제·검사에는 세 모듈의 위 구현이 모두 필요합니다. 학습된 ACT provider·영상 전처리, 실측 프로파일·하드웨어 정지, 운영 DB/outbox·재시작 조정, 현장 MQTT ACL은 별도 연결해야 합니다. core CI와 달리 이 통합 검사는 임시 loopback 소켓을 사용합니다.

## 관련 문서

이 디렉토리는 단일 저장소의 구성요소입니다. 아래 명령은 이 디렉토리에서 실행합니다.

- [프로젝트 개요](https://github.com/kernel-robot-8th/kernel-robot-repo-1)
- [요구사항·설계 문서](../../docs/README.md)
- [Jira](https://fp-team1.atlassian.net/jira/projects/FFT1)

PR에 관련 Jira 키, 변경 내용, 검증 결과와 미검증 항목을 기록합니다.
실로봇 적용 전 장비·설정·캘리브레이션, 안전 정지·복구 절차를 확인합니다.

## 개발 방법

```bash
python3 -m pip install uv==0.13.5
uv sync --locked
uv run ruff check src
uv run python -c "import roport_omx"
uv build
```

## LeRobot 원본

[lerobot/](lerobot/)에 공식 `v0.6.1` 소스를 포함했습니다. 원본 커밋과 라이선스는 [UPSTREAM.md](UPSTREAM.md)에 기록합니다.

기존 코어 패키지와 별도 환경에서 설치합니다. 다음 명령은 소스 설치만 수행하며 로봇을 구동하지 않습니다.

```bash
cd lerobot
python -m pip install -e .
```

기본 CI 실행: `bash ci/test.sh` — 원본 구문·CLI 진입 함수·encoder 변환과 장비 검사 로직을 검증합니다.

장비 연결 검사는 전용 Python 환경에 `dynamixel-sdk==3.8.4 pyserial==3.5`를 설치한 뒤 실행합니다.

```bash
python scripts/check_hardware.py --port /dev/serial/by-id/YOUR_OMX_DEVICE --output /tmp/omx-check.json
```

기존 제어 프로그램이 시리얼 포트를 사용 중이면 함께 실행하지 마세요. PING·READ로 ID 11–16의 모델·현재 위치(encoder count)·토크·오류 상태만 읽습니다. 모터 설정·토크를 변경하지 않으며, 주행·관절 동작·캘리브레이션 정확도는 별도 현장 검증이 필요합니다. 검사 실패 시 exit code 1과 JSON 결과를 남깁니다.
