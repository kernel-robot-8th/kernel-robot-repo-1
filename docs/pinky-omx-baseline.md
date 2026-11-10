# Pinky·OMX 공통 시험 배포 기준

이 문서를 PC별 설치·실행의 시작점으로 사용한다. 이전 채팅의 경로와 명령을 섞지 않는다.

## 1. 공통 버전과 적용 범위

| 항목 | 기준 |
| --- | --- |
| 공통 브랜치 | `feature/inyup-baek/pinky-omx-baseline` |
| 실행 소스 기준 | `111984aa4197cfc20afb5959342e3e94014131d3` |
| 구성 | main `706ad60` + PR55 `22a23cb` 통합 |
| 배포 커밋 | 관제 PC에서 한 번 기록한 `deployment.commit`; 모든 PC가 같은 값 사용 |
| 통신 계약 | Pinky `roport-pinky-driving-trial-v1`, OMX `roport-omx-task-v1`, 표시 `roport-state-dev-v1` |
| 환경 | 관제·OMX Python 3.12 + 각 uv.lock, 자산 준비 Node 24, Pinky Ubuntu 24.04 / ROS Jazzy |

**공통 시험 기준이며, 실물 OMX 자동 작업을 인수한 최종 릴리스는 아니다.** PR55는 main 병합 전이다.
문서/검사 도구 변경으로 배포 커밋이 실행 소스 기준과 달라도 서비스 내용이 같으면 검사에 통과한다.
브랜치 이름만 같다고 같은 코드가 아니다. 시험 중 `git pull`하지 않는다.

가능한 범위는 격리 가상 Pinky+mock OMX, LAN OMX 무구동 상태 왕복, 현장 승인 후 Pinky 주행·수동 OMX 완료다.
실물 OMX 실행 프로그램/모델/보정, AI 환경, PC별 기존 설치 경로와 현장 인수는 아직 미확인이다.

최신 main 통합으로 지도 이미지와 ArUco 배치가 이전 PR55 단독본과 달라졌다.
과거 trial.json의 map_version/현장 확인을 그대로 승인하지 않는다. 아래 새 기준 지도·layout으로 설정을 생성하고
현장에서 재확인한다. 확인 전 `field_verified=false`를 유지한다.

## 2. PC 역할과 고정 경로

| PC | 실행 프로세스 | 적용 코드 |
| --- | --- | --- |
| 브로커 PC | 기존 MQTT 브로커 1개 | 기존 서비스·인증·ACL 유지; 버전은 현장 기록 |
| 관제 PC | TrialControl 1개 + FastAPI 1개 | 공통 브랜치의 roport-control-server |
| 각 Pinky | 하위 구동부 + trial stack + MQTT runner | 같은 브랜치의 roport-pinky + 같은 소스로 만든 MQTT 키트 |
| OMX Host PC | cooperative_host.py 1개 | 같은 브랜치의 roport-omx, 현재 mock |
| AI PC | 무구동 시험에서는 불필요 | 실물 실행부 확정 후 환경·모델·시작 명령 별도 기록 |

각 PC의 사용자 이름은 달라도 아래 상대 위치는 통일한다.

```text
~/roport-baseline/
  repo/                  # 공통 배포 커밋 checkout
  run/
    deployment.commit    # 모든 PC 동일한 전체 SHA
    trial.json           # 실물 Pinky + 수동 OMX용, 최초 생성 시 미승인
    trial-omx-link.json   # OMX mock LAN 상태 조회용, 실물 주행에 사용 금지
    broker.env           # 각 PC의 MQTT 접속값, Git/배포 ZIP에 넣지 않음
    robot.env            # Pinky별 ROS/기체 설정, Git 밖에서 관리
    trial.sock           # 관제 PC만
    *.json / *.jsonl      # 역할별 journal/기록, 삭제하여 복구 우회 금지
  kit/                   # Pinky 새 키트, 구형 ~/roport_trial과 섞지 않음
  venv/                  # Pinky 전용 Python 환경
```

## 3. 최초 코드 설치와 배포 커밋 고정

**관제 PC에서 한 번:** 기존 checkout은 보존하고 새 디렉터리를 사용한다. 이미 존재하면 clone을 반복하지 않는다.

```bash
mkdir -p "$HOME/roport-baseline/run"
git clone --branch feature/inyup-baek/pinky-omx-baseline \
  https://github.com/kernel-robot-8th/kernel-robot-repo-1.git \
  "$HOME/roport-baseline/repo"
git -C "$HOME/roport-baseline/repo" rev-parse HEAD \
  > "$HOME/roport-baseline/run/deployment.commit"
git -C "$HOME/roport-baseline/repo" checkout --detach \
  "$(cat "$HOME/roport-baseline/run/deployment.commit")"
```

**OMX PC·각 Pinky:** 같은 clone 명령으로 코드를 받는다. 관제 PC에서 받은 deployment.commit을 run/에 놓은 후
아래를 실행한다. 각 PC가 자기 HEAD로 배포 커밋 파일을 새로 만들지 않는다.

```bash
git -C "$HOME/roport-baseline/repo" fetch origin
git -C "$HOME/roport-baseline/repo" checkout --detach \
  "$(cat "$HOME/roport-baseline/run/deployment.commit")"
git -C "$HOME/roport-baseline/repo" rev-parse HEAD
```

출력 SHA가 관제 PC와 같아야 한다. 이후 버전 검사에서 MISMATCH면 진행하지 않는다.
검사 도구가 없는 예전 커밋이면 배포 커밋/브랜치를 잘못 받은 것이다.

## 4. 관제 PC: 의존성·시험 설정·키트 준비

Python 3.12, uv, Node 24가 설치된 환경에서 수행한다. 기존 시스템 ROS/Python을 교체하지 않는다.

```bash
cd "$HOME/roport-baseline/repo/services/roport-control-server"
uv sync --locked
npm ci --ignore-scripts
npm run prepare:assets
uv run python tools/prepare_pinky_trial.py \
  --map ../roport-pinky/maps/airport_map.yaml \
  --layout ../roport-pinky/src/roport_gz_world/layout/roport_airport.yaml \
  --bundle-dir "$HOME/roport-baseline/run/candidates"
cp -n "$HOME/roport-baseline/run/candidates/single/hardware/trial.json" \
  "$HOME/roport-baseline/run/trial.json"
```

초기 기준은 물류 Pinky **1대(single)**다. 2대 시험은 two/hardware에서 생성한 동일 파일로 모든 PC가 별도 시험을
시작한다. 서로 다른 설정의 site/binding을 섞지 않는다. candidates는 각 PC에서 다시 생성하지 않는다.

편집기로 trial.json의 routes별 speed_limit_mps를 0.03으로 맞춘다. 출발 위치/방향·정차·지도·watchdog을
현장에서 확인한 뒤에만 운영자가 field_verified=true로 바꾼다. 아래 정적 검사는 물리 승인이 아니다.

```bash
uv run python -m roport_control_server.executor.pinky_trial \
  --config "$HOME/roport-baseline/run/trial.json" --check --scope line-only
uv run python tools/configure_omx_trial.py \
  --config "$HOME/roport-baseline/run/trial.json" \
  --output "$HOME/roport-baseline/run/trial-omx-link.json" \
  --execution-mode mock --slot 2b \
  --model-version test-model --calibration-version test-cal
uv run python tools/build_pinky_mqtt_kit.py \
  --output "$HOME/roport-baseline/run/pinky-mqtt.zip"
```

아래 파일을 해당 PC의 run/으로 복사한다. broker.env·UI 키·journal·모델/보정 파일은 배포 ZIP에 넣지 않는다.

| 대상 | 복사할 파일 |
| --- | --- |
| 각 Pinky | deployment.commit, trial.json, pinky-mqtt.zip |
| OMX Host | deployment.commit, trial-omx-link.json |

설정을 편집했다면 재배포하고 같은 용도의 파일끼리 sha256sum을 비교한다.
trial.json과 trial-omx-link.json은 용도가 다르므로 서로의 해시 일치는 요구하지 않는다.

## 5. PC별 버전 검사

```bash
# 관제 PC
python3 "$HOME/roport-baseline/repo/scripts/check_trial_baseline.py" \
  --repo "$HOME/roport-baseline/repo" --role control \
  --config "$HOME/roport-baseline/run/trial.json"
# OMX PC
python3 "$HOME/roport-baseline/repo/scripts/check_trial_baseline.py" \
  --repo "$HOME/roport-baseline/repo" --role omx \
  --config "$HOME/roport-baseline/run/trial-omx-link.json"
# Pinky: 7절의 키트 설치 후 실행
python3 "$HOME/roport-baseline/repo/scripts/check_trial_baseline.py" \
  --repo "$HOME/roport-baseline/repo" --role pinky \
  --config "$HOME/roport-baseline/run/trial.json" --kit "$HOME/roport-baseline/kit"
```

result=PASS, 동일 checkout_head, 같은 용도의 config_sha256 일치가 필요하다.
Pinky는 ZIP manifest의 자체 주장만 확인하지 않고 배포된 Python 8개 파일을 기준 Git 소스와 직접 비교한다.
MISMATCH는 차이를 확인한 뒤 수정한다. 현장 보정을 지우는 git reset --hard는 사용하지 않는다.
PASS는 소스 일치만 뜻한다. ROS overlay·설치된 의존성·ignored 파일·브로커·실물은 별도 확인이다.
검사 도구는 드라이버를 import하거나 네트워크/모터에 접근하지 않는다.

## 6. 공통 브로커와 OMX 무구동 시험

브로커 PC의 기존 서비스 1개를 유지한다. 다른 PC에 별도 브로커를 추가하지 않는다.
실제 LAN 주소/포트는 현장 broker.env에 지정한다. 아래 자리표시자를 그대로 사용하지 않는다.

```bash
export ROPORT_MQTT_HOST=REPLACE_WITH_BROKER_LAN_HOST
export ROPORT_MQTT_PORT=1883
export ROPORT_MQTT_TLS=0  # 기존이 비TLS·무인증 시험 구성일 때만
```

인증/TLS를 사용 중이면 기존 설정을 각 PC에 적용한다. 암호 인증은 TLS가 필요하다.
CA 경로는 각 PC에서 유효해야 한다. broker.env는 Git 밖에 둔다.
ACL에는 site의 workcell/shelf-01/cmd,event,ack를 추가한다.
관제는 cmd/ack 발행·event 구독, OMX는 cmd/ack 구독·event 발행이며 QoS1, retain=false다.

**OMX PC의 Host 터미널:**

```bash
cd "$HOME/roport-baseline/repo/services/roport-omx"
uv sync --locked
source "$HOME/roport-baseline/run/broker.env"
read -r SITE_ID CONTROL_ID OMX_BINDING < <(
  python3 - <<'PY'
import json
from pathlib import Path
p = json.loads((Path.home() / 'roport-baseline/run/trial-omx-link.json').read_text())
print(p['site_id'], p['control_id'], p['omx']['binding_id'])
PY
)
uv run python scripts/cooperative_host.py \
  --site "$SITE_ID" --control "$CONTROL_ID" --binding "$OMX_BINDING" \
  --client-id roport-shelf-01-host \
  --journal "$HOME/roport-baseline/run/omx-mock.json"
```

정상 로그는 SIMULATED executor와 subscribed다. 이 Host는 실제 팔을 움직이지 않는다.
**관제 PC의 조회 터미널:** 같은 roport-omx 디렉터리에서 uv sync, broker.env 읽기, 위 ID 읽기를 수행한 뒤
아래를 실행한다. TrialControl owner는 종료된 상태여야 한다.

```bash
uv run python scripts/cooperative_control_sim.py status \
  --site "$SITE_ID" --control "$CONTROL_ID" --binding "$OMX_BINDING" \
  --timeout 15 --log "$HOME/roport-baseline/run/omx-status.jsonl"
```

성공 기준은 status.completed다. 기존 조회 스크립트는 TLS/인증 환경변수를 사용하지 않아 무인증·비TLS 시험 전용이다.
인증이 있는 현장에서 인증을 끄지 않는다. 해당 경우 인증을 지원하는 조회 클라이언트의 별도 확인이 필요하다.
이 단계에는 Pinky bringup·AI·관제 UI 출발이 필요 없다.

## 7. Pinky: 동일 코드와 새 키트 설치

Ubuntu24.04/ROS Jazzy와 검증된 하위 모터/센서·IR 보정이 전제다. 기존 하위 구동 환경의 경로가 불명확하면
현장 담당자가 먼저 확인한다. 보정값을 다른 Pinky에서 복사하지 않는다.

```bash
cd "$HOME/roport-baseline"
python3 - <<'PY'
from pathlib import Path
from zipfile import ZipFile
p = Path.home() / 'roport-baseline'
(p / 'kit').mkdir(exist_ok=False)
with ZipFile(p / 'run/pinky-mqtt.zip') as z:
    z.extractall(p / 'kit')
PY
python3 -m venv --system-site-packages "$HOME/roport-baseline/venv"
"$HOME/roport-baseline/venv/bin/python" -m pip install \
  -r "$HOME/roport-baseline/kit/requirements.txt"
source /opt/ros/jazzy/setup.bash
# 필요한 하위 구동 패키지는 기존 검증된 workspace의 setup.bash를 먼저 source한다.
cd "$HOME/roport-baseline/repo/services/roport-pinky"
colcon build --packages-up-to roport_pinky
source install/setup.bash
ros2 pkg prefix roport_pinky
```

prefix가 이번 ~/roport-baseline/repo/services/roport-pinky/install/ 아래인지 확인한다.
의존성 부족은 [Pinky README](../services/roport-pinky/src/roport_pinky/README.md)의 rosdep 대상을 확인한다.
키트는 기존 폴더에 덮어쓰지 않고 새 kit/에 설치한다. 설치 후 5절의 버전 검사를 실행한다.

각 Pinky의 run/robot.env에 아래 현장값을 저장한다. 다른 Pinky에는 다른 ROS_DOMAIN_ID를 사용한다.

```bash
export ROS_DOMAIN_ID=REPLACE_WITH_ASSIGNED_DOMAIN
export ROBOT_NAME=REPLACE_WITH_ROBOTS_YAML_KEY
export ROPORT_PINKY_START_NODE=N16  # single/logistics의 실제 배치 위치
export ROPORT_PINKY_MAP="$HOME/roport-baseline/repo/services/roport-pinky/maps/airport_map.yaml"
export ROPORT_PINKY_LAYOUT="$HOME/roport-baseline/repo/services/roport-pinky/src/roport_gz_world/layout/roport_airport.yaml"
```

## 8. 실물 Pinky + 수동 OMX 완료 실행

현장 정지 수단·배치·새 지도를 확인하고 trial.json의 field_verified를 승인한 뒤에만 수행한다.
현재 가능한 물리 시험은 **OMX 설정 없는 trial.json**이다. mock 완료를 실제 상하차 근거로 사용하지 않는다.
각 Pinky 터미널에서 ROS Jazzy·필요한 하위 workspace·이번 install/setup.bash·robot.env를 같은 순서로 source한다.

| Pinky 터미널 | 명령 |
| --- | --- |
| A: 하위 bringup | ros2 launch pinky_bringup bringup_robot.launch.xml — 이미 실행 중이면 중복 실행하지 않음 |
| B: trial stack | 아래 launch. 기존 line/Nav2/teleop을 함께 실행하지 않음 |
| C: MQTT runner | 아래 runner. 기체당 1개 |

```bash
# B: 직진 0.03m/s, 회전 0.15rad/s
ros2 launch roport_pinky trial.launch.py \
  robot:="$ROBOT_NAME" map:="$ROPORT_PINKY_MAP" layout_file:="$ROPORT_PINKY_LAYOUT" \
  line_speed:=0.03 line_rotate_speed:=0.15 max_linear:=0.03 max_angular:=0.15 ir_node:=true
```

IR 노드가 이미 실행 중이면 ir_node:=false를 사용한다. 터미널 C:

```bash
source "$HOME/roport-baseline/run/broker.env"
cd "$HOME/roport-baseline/kit"
"$HOME/roport-baseline/venv/bin/python" -m pinky_mqtt.trial_runner \
  --config "$HOME/roport-baseline/run/trial.json" --role logistics \
  --journal "$HOME/roport-baseline/run/logistics_$(date +%Y%m%d_%H%M%S).json" \
  --driver roport_pinky.trial_driver:create_driver --hardware-authorized
```

**관제 PC 터미널 A:**

```bash
cd "$HOME/roport-baseline/repo/services/roport-control-server"
source "$HOME/roport-baseline/run/broker.env"
uv run python -m roport_control_server.executor.pinky_trial \
  --config "$HOME/roport-baseline/run/trial.json" \
  --journal "$HOME/roport-baseline/run/control_$(date +%Y%m%d_%H%M%S).json" \
  --socket "$HOME/roport-baseline/run/trial.sock" --scope line-only --hardware-authorized
```

**관제 PC 터미널 B:**

```bash
cd "$HOME/roport-baseline/repo/services/roport-control-server"
export ROPORT_MAP="$(realpath ../roport-pinky/maps/airport_map.yaml)"
export ROPORT_TRIAL_LAYOUT="$(realpath ../roport-pinky/src/roport_gz_world/layout/roport_airport.yaml)"
export ROPORT_TRIAL_SOCKET="$HOME/roport-baseline/run/trial.sock"
read -rsp '웹 조작 키 24자 이상: ' ROPORT_TRIAL_TOKEN
export ROPORT_TRIAL_TOKEN
uv run uvicorn roport_control_server.adapters.api.app:app \
  --host 127.0.0.1 --port 8000 --ws-max-size 4096 --ws-max-queue 4
```

같은 PC에서 http://127.0.0.1:8000/ui/ 를 연다. 서비스 시험은 계획 확인→적용→별도 실행 순서다.
실제 물품 전달과 팔 복귀를 작업자가 확인하고 수동 완료를 입력한다. 다른 PC의 HTTP 접속은 실물 조작을 차단한다.
원격 조작은 인증된 HTTPS 구성으로 별도 검증한다.

## 9. 가상 통합 시험과 실물 자동 연동의 경계

전체 소프트웨어 흐름 확인은 관제 PC에서 아래를 사용한다. 기존 LAN broker와 격리된다.

```bash
cd "$HOME/roport-baseline/repo/services/roport-control-server"
uv run python tools/run_virtual_trial.py \
  --scenario two --scope line-only --omx mock \
  --output "$HOME/roport-baseline/virtual-$(date +%Y%m%d_%H%M%S)"
```

Mosquitto 실행 파일이 필요하다. PATH에 없으면 --mosquitto /absolute/path/to/mosquitto를 지정한다.
UI에서 입고/출고 계획과 MQTT 완료를 확인한다. 가상 설정은 loopback 전용이며 LAN용으로 mode를 바꾸지 않는다.

실물 OMX 자동 연동은 아래가 확인될 때 실행 절차를 확정한다.

1. 실행 프로그램의 Git SHA·경로·시작 명령·의존 환경과 모터 단독 소유.
2. 최신 readiness: armed/busy/error, 양팔 connected/returned/safe/grasping.
3. 작업/slot별 모델·보정·상자 marker, start/observe의 동일 실행 결과 확인.
4. 실물 정지·실패·복귀 검증과 Pinky 재출발 조건의 현장 승인.

FFT1-158 panel 후보 d398ba5는 readiness의 arms가 비어 현재 관제 준비 조건을 통과하지 못한다.
각 PC가 그 브랜치를 임의로 섞지 않는다. 담당자의 수정본을 공통 브랜치에 리뷰 후 통합하고 기준을 갱신한다.
OMX3 MQTT 상차는 범위 밖이며 수동이다. AI·모델·보정이 이 Git SHA만으로 관리된다고 간주하지 않는다.

## 10. 종료·재시험·업데이트

1. 작업을 완료하거나 UI 시험 중단 후 현장에서 Pinky와 팔의 정지를 확인한다. UI 중단은 물리 비상정지의 대체가 아니다.
2. 관제 owner를 Ctrl+C로 종료한다. Pinky runner·trial stack·하위 bringup을 순서대로 종료한다.
3. OMX Host를 종료한다. 실물 실행부는 현장 정지 절차를 따른다. API를 마지막에 종료하고 공유 브로커는 유지한다.
4. journal/HOLD를 보존한다. 결과 불명이면 물품·자세를 확인한 뒤 새 시험을 만든다. 삭제나 --clear-hold만으로 재개하지 않는다.
5. 업데이트는 별도 checkout/run에 배포 커밋·설정·키트를 함께 적용하고 각 PC에서 5절을 다시 수행한다.

정지 후 재연결로 guard의 lease/session이 바뀌면 trial stack도 다시 시작한다.
이전 journal을 새 버전/설정에 재사용하지 않는다. 이전 환경은 되돌리기용으로 보관하되 자동 재개하지 않는다.

## 11. 인수 기록

각 PC의 deployment.commit, 버전 검사 JSON, 같은 용도의 설정 SHA256, Python/ROS/의존 버전,
실제 ROS prefix와 시작 로그를 로컬에 보관한다. env·카메라·민감한 로그를 공개 PR에 올리지 않는다.
브로커 버전/설정/ACL, 실물 OMX/AI/모델/보정의 위치는 현재 미확인이다.
소스 PASS·MQTT status 성공·가상 COMPLETE·실물 작업 성공을 별도로 기록한다.
