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
