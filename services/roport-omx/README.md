# roport-omx

RO:PORT · GATE CREW — OMX 제어·협동 동작·시연 수집·학습

## 기술 스택

Python 3.12 코어 패키지 / uv / Ruff. 학습·정책 실행은 LeRobot·PyTorch를 사용하되 OMX 드라이버·CUDA 호환성을 확인한 별도 환경에서 버전을 고정합니다.

## 현재 범위

팔별 분담 Host와 패널 작업 런타임, 시연 데이터 검사·ACT 학습 실행, 관측별 영상 전송, 감독형 수집·평가 어댑터를 제공합니다. 기본 검사는 합성 데이터·fake 장비·loopback을 사용합니다. GPU 학습과 실제 장비 인수는 별도 확인이 필요합니다.
CI 성공은 실로봇·GPU·통합 검증 완료를 의미하지 않습니다.
배포 환경 미정으로 CD와 자동 배포는 설정하지 않았습니다.

## 팔별 분담 실행 코어

`src/roport_omx/dedicated.py`의 `Profile`과 `DedicatedHost`가 입고·반출 담당 팔, 단일 포장품·슬롯, 정차 permit, 양팔 관측, 정책·보정 버전을 검사합니다. SQLite에 명령을 저장한 뒤 접수하고 같은 명령은 재실행하지 않습니다. 재시작 시 불명확한 실행은 `RECONCILING`으로 남깁니다.

`GRASPING → WAIT_GRASP → TRANSFERRING → WAIT_SEATED → RELEASING → WAIT_RELEASE → RETURNING → WAIT_SAFE → SAFE_RETURNED` 순서로 진행합니다. 단계 경계와 다음 정책 시작 시 AI reset 응답을 확인하며, 정상 대기와 오류 보류를 구분합니다. 최신 관측·확인 입력 없이 chunk 종료나 그리퍼 위치만으로 완료하지 않습니다. Host는 출발을 허가하지 않습니다.

- `tick()`은 내부 타이머가 없는 호출형 watchdog입니다. 호출 측이 idle·`WAIT_*`를 포함해 실행 중 계속 주기적으로 호출해야 lease·deadline·대기 timeout이 `HOLD`로 반영됩니다. 호출 주기는 현장 프로파일에서 정하며 기본값이 없습니다.
- `WAIT_*`에서 `next_action()`·`infer_request()`는 `HOLD` 없이 `None`을 반환해 이동만 막습니다. timeout·permit 철회는 그대로 `HOLD`입니다. `HOLD`의 `reason`은 최초 원인을 유지합니다.
- `confirm()` 입력: `execution_id`, `phase`, `phase_revision`, `verdict_kind`(`WAIT_GRASP`→`GRASPED`, `WAIT_SEATED`→`SEATED`, `WAIT_RELEASE`→`RELEASED`, `WAIT_SAFE`→`SAFE`), `confirmer`, `positive`, `observation_seq`, `captured_at`, 마지막 대기 전에는 `next_policy_id`. 명세의 `observed_at`은 `captured_at`(확인에 묶인 관측 시각)입니다. 소비한 확인은 journal `confirmations` 테이블에 기록합니다.
- 같은 `command_id`·같은 내용은 만료 후에도 저장 상태/결과를 반환하고, 내용이 다르면 `Rejected.code == "IDEMPOTENCY_CONFLICT"`로 거절하며 진행 중 실행은 바꾸지 않습니다. 같은 실행·boot·도킹의 이전 `evidence_seq` 재전달은 무시합니다.

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

초기 개발은 `task/omx` 아래 네 브랜치로 나눴습니다. 후속 패널 실행 연동은 이들의 통합 커밋을 기반으로 한 `feature/jsh0116/omx-panel-runtime`에서 진행합니다.

- `feature/jsh0116/omx-dedicated-control`: 관제 명령·permit·P7 작업 순서.
- `feature/jsh0116/omx-dedicated-host`: Host 프로파일·상태기계·SQLite 이력과 단독 mock 예제.
- `feature/jsh0116/omx-dedicated-ai`: 정책 라우터·UDP adapter와 wheel 구성.
- `feature/jsh0116/omx-dedicated-integration`: 위 세 브랜치를 합친 연결 예제와 회귀 검사. 검토할 때는 각 기능 브랜치부터 확인하고 통합 브랜치에서 연결 변경을 확인합니다.

의존성: 통합 예제·검사에는 세 모듈의 위 구현이 모두 필요합니다. ACT provider·영상 연결은 후속 패널 런타임에 포함합니다. 실제 체크포인트·실측 프로파일·물리 정지 callback·운영 DB/outbox·현장 MQTT ACL은 배포 환경에서 연결하고 검증해야 합니다. core CI와 달리 이 통합 검사는 임시 loopback 소켓을 사용합니다.

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

## 패널 실행 연동

패널 UI와 웹 서버는 팀원 저장소에서 유지합니다. 이 모듈은 실행 계약과 이력, 데이터 수집·ACT 학습·감독형 평가를 관리합니다. 외부 패널 v3.3.0의 확인 기준은 `b058e777025a779b23b7a930222b5b5ebdcffec5`입니다. 패널 hook은 외부 저장소에 따로 반영해야 하므로, 아래 구현만으로 실제 GUI 연동을 완료했다고 볼 수 없습니다.

### 패널 서버에서 연결할 부분

`roport_omx.panel_runtime`의 `PanelRuntime`과 `panel_request`, `workflow_handlers`를 사용합니다. 패널 서버가 런타임 인스턴스 하나를 유지하고, 아래 요청을 기존 subprocess 실행 대신 위임합니다.

- `/api/record` → `record`, `/api/train/start` → `train`, `/api/eval/start` → `evaluate`.
- `panel_request(route, payload, request_id=..., profile_id=..., profile=...)`로 변환한 뒤 `runtime.submit(request)`를 호출합니다. 클릭 재전달에도 같은 `request_id`를 유지합니다.
- 패널의 상태·중단 요청을 `runtime.status(job_id)`와 `runtime.control(job_id, "stop")`에 연결합니다. 자동 resume은 지원하지 않습니다.
- 첫 연동은 단일 데이터셋·단계별 ACT만 지원합니다. 학습 form의 `datasets=[{"repo": "등록한 별칭", "exclude": []}]`, `name`, `steps`, `batch_size`를 변환합니다. `num_workers=0`, `augmentation=false`, 빈 `note`만 허용합니다. `phase/seed/chunk_size/n_action_steps`는 선택한 신뢰 프로파일에 명시합니다. Diffusion/SmolVLA·부분 episode 제외·병합·임의 인수는 거부합니다.
- 녹화와 평가는 승인한 `profile.panel_parameters`의 form 값과 정확히 일치해야 합니다. 평가는 임의 `model_path` 대신 등록된 `preset_id`를 선택하도록 hook을 바꿉니다. unsupported 필드를 조용히 무시하지 말고 패널에 오류로 표시합니다.
- 기존 패널 자체 구동과 Host도 같은 `shared_lock_path`와 `RECONCILING` marker 규칙을 사용해야 합니다. 외부 패널의 기존 잠금 코드가 오류를 무시하는 경로로 장비를 열지 않도록 수정해야 합니다.

호출 계약은 다음과 같습니다. 예시 ID는 합성 값입니다.

```json
{
  "schema_version": "omx.workflow/1",
  "request_id": "synthetic-request-1",
  "operation": "train",
  "profile_id": "synthetic-inbound-grasp",
  "parameters": {
    "dataset_id": "demo", "output_id": "act-grasp-v1", "phase": "GRASPING",
    "steps": 100, "batch_size": 1, "seed": 7,
    "chunk_size": 2, "n_action_steps": 1
  }
}
```

숫자는 API 예시이며 실제 학습 설정·현장 제어 수치가 아닙니다. 각 프로파일은 `operations` 허용 목록을 명시합니다. handler는 `(request, profile, job_dir, stop_event) -> dict`이며, `workflow_handlers(devices_factory=..., host_factory=...)`가 실제 모듈 함수를 연결합니다. factory를 주지 않으면 장비 작업을 등록하지 않습니다. 프로파일·factory·callback은 배포 코드에서만 주입하고 HTTP 요청으로 받지 않습니다.

SQLite는 접수·상태·결과·오류를 저장하고 한 DB의 실행 소유자는 하나입니다. 같은 ID·같은 요청은 기존 job을 반환하고 다른 내용은 `IDEMPOTENCY_CONFLICT`입니다. 실행은 직렬이며 서버 재시작 후 미완료 job은 `RECONCILING`입니다. 새 장비 작업은 해결 전까지 차단합니다. `reconcile(job_id)`는 신뢰 프로파일의 `reconcile_job` callback이 실제 상태 확인 근거를 반환할 때만 이전 job을 실패 종료하며 재실행하지 않습니다. callback에서 장비 lock의 물리 복구도 확인해야 합니다. `close()`는 작업 종료를 기다리며 모터 프로세스를 강제 종료하지 않습니다.

### 데이터·학습 계약

`learning.inspect_dataset`은 LeRobot v3 데이터와 `manifest.json`의 `omx.dataset/1`을 검사합니다. manifest에는 아래 내용을 기록합니다.

- `profile`: operation·arm_id(`omx1`/`omx2`), 순서가 고정된 6개 feature·단위, schema/calibration 버전, FPS·명시한 timestamp 오차, RGB HWC 카메라 shape.
- `episodes`: 원본 시연·session·condition group ID, 슬롯·포장품·정차 편차(mm/degree), 성공/실패 라벨, split, 완료 여부, frame 수와 영상 파일 목록.
- 네 phase의 시작/끝 frame(끝은 미포함)과 감독자/센서 판정 근거, frame별 관측·실제 action·카메라 시각(ns).

단위 식별자는 기존 Host와 같은 `range_m100_100` 또는 `degrees`(팔 5관절), `range_0_100`(그리퍼)입니다. 방향·보정과 ROS m/rad 변환을 암묵적으로 바꾸지 않습니다. 같은 source/session/condition의 split 누출, 단계 중첩·경계 초과, 불완전 영상·메타데이터, timestamp/FPS 불일치를 거부합니다. 파일 검사만으로 영상 내용을 모두 디코딩했다는 뜻은 아닙니다. 준비 단계에서 LeRobot API로 실제 frame을 읽고 새 phase episode를 저장하며, 실패 시연은 별도 검수 전 train에 넣지 않습니다.

`prepare_phase_dataset`은 train/validation/evaluation을 따로 저장합니다. 학습에는 train 통계만 쓰며 chunk가 다른 단계의 frame으로 넘어가지 않습니다. `run_training`은 고정된 Python 환경을 `shell=False`로 실행하고 ACT·Hub/W&B 비활성화·기존 출력 보존을 강제합니다. 독립 프로세스의 같은 output ID도 원자 잠금으로 차단합니다. exit code 0뿐 아니라 checkpoint와 저장된 normalizer/unnormalizer state가 있어야 성공입니다. `metadata.json`은 모델·관절·카메라·데이터 provenance와 미실행 평가 상태를 AI provider에 인계합니다. 전체 작업을 학습한 실습 모델을 네 단계 정책으로 자동 등록하지 않습니다.

프로파일과 요청 JSON은 저장소 밖에 둡니다. runtime profile에는 `dataset_root`, `checkpoint_root`, `expected_profile`, `python_executable`, `learning_cli`, `device`, `runtime_version`, 실제 설치 `environment`, `model_metadata.policy_bundle_version`을 명시합니다. 상대 경로 대신 실제 절대 경로를 사용합니다. CLI는 장비를 연결하지 않습니다.

```bash
python scripts/learning_cli.py --help
python scripts/learning_cli.py inspect --root "$DATASET_ROOT" --expected-profile "$EXPECTED_PROFILE_JSON"
python scripts/learning_cli.py train --request "$TRAIN_REQUEST_JSON" --runtime-profile "$RUNTIME_PROFILE_JSON" --job-dir "$NEW_JOB_DIRECTORY"
```

마지막 명령은 실제 학습을 실행합니다. 검수한 데이터와 새 output ID, 선택한 CPU/GPU 환경을 준비한 뒤 실행합니다. 변수 값은 각 환경에서 먼저 설정해야 합니다. 현장 데이터·가중치·설치 버전 원본은 커밋하지 않습니다.

### 제어 PC의 수집·평가

`lerobot_runtime.record/evaluate`는 명시한 trusted profile과 장비 어댑터를 받습니다. `LeRobotDevices`와 `LeRobotDatasetWriter`는 LeRobot을 지연 로드하며, profile이 불완전하면 시작하지 않습니다. `authorization.supervisor`, 하드웨어 승인, 검증한 보정파일 SHA/version·양팔 설정·startup 검증·physical stop/support·timestamp 결속 callback이 필요합니다. 원본 `connect(calibrate=False)`도 설정·토크 관련 쓰기를 할 수 있으므로 연결 확인 명령으로 사용하지 않습니다.

원본 OMX Leader는 정규화 단위이므로 실제 수집 어댑터는 `use_degrees=true`를 거부합니다. 검증된 변환 없이 Leader 정규화 값을 degree Follower로 보내지 않습니다. 녹화는 clipping 후 `send_action()`이 반환한 실제 목표를 기록합니다. 평가는 기존 Host의 후보 검사와 `next_action()`을 통과하고 inactive arm·permit을 매 적용 전에 확인합니다. 현재 수집/평가 길이와 task 등은 승인된 프로파일에서 선택하며 임의 panel parameter로 덮어쓰지 않습니다.

중단·예외·부분 연결 실패에서는 물품 지지와 물리 정지 callback의 근거가 필요합니다. 확인하지 못하면 lock marker를 유지하고 `RECONCILING`으로 남깁니다. 프로세스 종료로 OS 잠금이 풀려도 marker가 재시작을 막습니다. `reconcile_devices`는 감독자 복구에서만 사용합니다. HOLD·stop 버튼·프로세스 종료를 물리 정지 완료로 표시하지 않습니다.

### GPU 노트북에서 확인할 순서

작업 중인 변경을 보존한 상태에서 새 브랜치를 가져옵니다. 이미 같은 로컬 브랜치가 있으면 새로 만들지 말고 그 브랜치를 선택하고 fast-forward 여부를 확인합니다.

```bash
cd ~/projects/kernel-robot-repo-1
git status --short
git fetch origin feature/jsh0116/omx-panel-runtime
git switch --track origin/feature/jsh0116/omx-panel-runtime
source ~/venv/omx-lerobot-v061/bin/activate
python -c 'import torch; print(torch.__version__, torch.version.cuda); assert torch.cuda.is_available(); x=torch.ones((64,64),device="cuda"); print(torch.cuda.get_device_name(0), (x@x).sum().item())'
python -m unittest discover -s services/roport-omx/ci -p 'test_*.py'
python services/roport-omx/scripts/learning_cli.py --help
```

먼저 합성/source/loopback 결과를 확인하고, 수집한 데이터의 manifest 검사 → phase 준비 → 작은 ACT 학습 → checkpoint/processor 재로딩·관측 replay 순서로 진행합니다. 실제 학습에는 위 환경의 editable LeRobot 설치가 필요하지만 다른 모델 extras를 추가 설치할 필요는 없습니다. 별도 코어 검사 환경은 모듈에서 `uv sync --locked`로 준비합니다. GPU 환경에 코어 의존성 전체를 섞지 않습니다.

제어 PC에서 GPU로 전송할 때는 실행 중인 녹화 디렉터리를 학습에 바로 노출하지 말고 임시 전송 경로에서 전체 파일·manifest 완료 상태를 검수한 뒤 새 dataset ID로 등록합니다. 기존 성공 데이터와 output은 덮어쓰지 않습니다. 실패 시 새 실행을 차단하고 원본 데이터·로그를 보존한 뒤, 승인된 이전 profile/checkpoint와 기존 가상환경으로 복귀합니다. 외부 패널 hook을 되돌릴 때도 기존 자체 실행과 Host가 동시에 장비를 소유하지 않는지 확인합니다.
