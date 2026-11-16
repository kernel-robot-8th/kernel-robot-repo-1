# Pinky 현장 적용 매뉴얼

기준: 2026-11-16 로컬 FLEET 개발본 / MQTT `schema_version=2.0`, `profile=roport-control-v2`.
[공통 준비](README.md)를 먼저 수행한다. 실물 주행 성공 이력을 기반으로 한 같은 TrialDriver를
사용하지만 최신 중간 정차·실시간 위치·N12 왕복 코드를 함께 설치해야 한다.

## 1. 필요한 정보

| 항목 | 입력 위치 / 기준 |
| --- | --- |
| 관제 브로커 IP·포트 | `current/broker.env`; Pinky IP가 아닌 브로커 PC 주소 |
| MQTT 기체명 | `pinky1`, `pinky2`, `pinky3`; 해당 폴더의 `trial.json` |
| 역할·초기 노드 | 기본 pinky1=logistics/N16, pinky2=logistics/N17, pinky3=smartcart/H1 |
| 실제 장비 이름 | `robot:=pinky_XXXX`; MQTT 이름과 다르며 해당 기체의 IR 보정 이름 |
| ROS domain | 기체별로 분리하고 그 기체의 모든 터미널에 같은 `ROS_DOMAIN_ID` 사용 |
| 지도·배치 | 제공된 `maps/airport_map.yaml` 및 이미지, `maps/layout.yaml` |

초기 노드는 실제 로봇을 놓은 위치다. 번호만 바꿔 위치를 가정하지 않는다. 이후 복귀 자리는
관제가 비어 있는 곳을 고른다. 역할을 바꾸면 endpoint의 초기 설정/배치를 같이 확인한다.

## 2. 이 버전 설치 — 기존 작업공간 보존

관제 PC에서 만든 폴더를 아래처럼 복사한다. SSH 계정·IP는 해당 기체의 값을 사용한다.

```bash
# 관제 PC. HANDOFF는 공통 문서에서 생성한 폴더
ssh pinky@PINKY_IP 'mkdir -p ~/roport-field/snapshot'
rsync -av "$HANDOFF/" pinky@PINKY_IP:~/roport-field/snapshot/
```

Pinky에서:

```bash
cd ~/roport-field/snapshot
sha256sum -c SHA256SUMS
cp -a ~/roport-field/snapshot ~/roport-field/current
export FIELD_DIR="$HOME/roport-field/current"
source /opt/ros/jazzy/setup.bash
# 해당 기체에서 기존에 검증된 ROS workspace를 지정
source "$HOME/colcon_ws/install/setup.bash"
mkdir -p "$HOME/roport-field/ros_ws/src"
cp -a "$FIELD_DIR/pinky-src/." "$HOME/roport-field/ros_ws/src/"
cd "$HOME/roport-field/ros_ws"
colcon build --packages-select roport_line_nav roport_pinky --symlink-install
source install/setup.bash
python3 -m venv --system-site-packages "$HOME/roport-field/venv"
"$HOME/roport-field/venv/bin/python" -m pip install -r "$FIELD_DIR/requirements.txt"
```

`snapshot/current/ros_ws`가 이미 있으면 위 복사를 반복해 덮어쓰지 말고 다른 버전 폴더를 만든다.
업데이트 시 같은 소스 위에 복사하면 제거된 파일이 남을 수 있다. 기존 `~/colcon_ws/src` 코드는
수정하지 않는다. 빌드에 필요한 ROS/IR/LiDAR/Nav2 패키지는 기존 실물 workspace가 제공해야 한다.
LeRobot·관제 서버 전체를 Pinky에 설치할 필요는 없다.

## 3. 현장 준비·ROS 실행

기체별 터미널 모두에서 ROS 환경과 **새 overlay**를 source한다. 예시는 Pinky1이다.
Pinky2는 ROBOT=pinky2, ROLE=logistics, START=N17;
Pinky3는 ROBOT=pinky3, ROLE=smartcart, START=H1로 바꾼다.

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/colcon_ws/install/setup.bash"
source "$HOME/roport-field/ros_ws/install/setup.bash"
export FIELD_DIR="$HOME/roport-field/current"
export ROBOT=pinky1
export ROLE=logistics
export START=N16
export ROS_DOMAIN_ID=31
source "$FIELD_DIR/broker.env"
export ROPORT_PINKY_START_NODE="$START"
export ROPORT_PINKY_MAP="$FIELD_DIR/maps/airport_map.yaml"
export ROPORT_PINKY_LAYOUT="$FIELD_DIR/maps/layout.yaml"
```

Pinky2/3는 사용 중이지 않은 다른 domain(예:32/33)을 사용한다. MQTT는 ROS domain과 무관하다.
기존 bringup·IR·LiDAR를 담당자의 검증된 절차로 준비한다. **bringup은 모터 초기화를 할 수
있으므로 이 매뉴얼의 읽기 전용 점검이 자동 실행하지 않는다.** 기존 line_nav·teleop·다른
Nav2·cmd_vel_guard는 중복 실행하지 않는다.

```bash
ros2 launch roport_pinky trial.launch.py robot:=REPLACE_WITH_CALIBRATION_NAME map:="$ROPORT_PINKY_MAP" layout_file:="$ROPORT_PINKY_LAYOUT" line_speed:=0.06 max_linear:=0.03 max_angular:=0.15
```

위 값은 라인 0.06m/s, Nav2 0.03m/s, Nav2 회전 0.15rad/s다. MQTT 제한을 올려도 ROS 상한이
낮으면 그 속도를 넘지 않는다. IR 노드가 아직 없을 때만 `ir_node:=true`를 추가한다.
AMCL 초기 위치를 실제 배치와 맞추고 N12/Nav2 시험에는 Nav2 노드를 켜둔다.

## 4. MQTT endpoint 실행

현재 위치·방향·지도·정지 감시를 현장에서 확인한 후 해당 `trial.json`의 `field_verified=true`를
설정한다. 기체 설정을 로봇마다 새로 생성하지 않는다. 관제와 동일한 버전/식별자를 유지한다.

```bash
cd "$FIELD_DIR"
mkdir -p "$HOME/.local/state/roport"
# 설정만 확인. 장치 factory를 import하거나 MQTT에 접속하지 않음
"$HOME/roport-field/venv/bin/python" -m pinky_mqtt.trial_runner --config "$FIELD_DIR/$ROBOT/trial.json" --role "$ROLE" --journal "$HOME/.local/state/roport/$ROBOT-field-v2.json" --hardware-authorized --check
# 현장 확인 후 실제 endpoint 실행
"$HOME/roport-field/venv/bin/python" -m pinky_mqtt.trial_runner --config "$FIELD_DIR/$ROBOT/trial.json" --role "$ROLE" --journal "$HOME/.local/state/roport/$ROBOT-field-v2.json" --driver roport_pinky.trial_driver:create_driver --hardware-authorized
```

`MQTT endpoint running ... (not motion readiness)`는 연결 표시다. 준비 완료 표시는 아니다.
관제 PC에서 공통 문서의 `check_field.py --listen`을 실행한다. Pinky1만 시험하면
`--targets pinky1`, 스마트카트까지 점검하면 `--targets pinky1 pinky3 --nav2 pinky3`로 지정한다.
UI에서 연결하지 않은 슬롯은 실행 전에 배정 제외한다. 통로에 실제 기체를 남겨놓고 빈자리로
간주하지 않는다.

## 5. 정상 기준·오류 대응

| 관측 | 정상 기준 / 확인할 것 |
| --- | --- |
| 연결이 안 됨 | 모든 PC의 broker IP/port/TLS, 방화벽, 동일 Wi-Fi, 중복 MQTT endpoint 확인 |
| `graph_version/map_version mismatch` | 제공 지도·이미지·layout와 해당 기체 JSON을 한 세트로 다시 대조 |
| `position/heading unknown` | START의 실제 배치, line_nav 최신 상태·odom 확인. 수동 좌표 입력으로 통과시키지 않음 |
| `drive owner not released` | 기존 주행/teleop 중복 여부, guard 상태 확인. 강제로 owner를 바꾸지 않음 |
| Nav2 미준비 | Nav2 lifecycle·map/TF/scan·초기 위치·지도 일치 확인. 단순 MQTT 연결은 무관 |

정지·위치 ROS 관측은 `ros2 topic echo /trial/localization_state --once`로 확인한다.
현재 위치는 최신 TF를 사용하고 라인 표시용 추정은 별도 구분한다. AMCL 메시지 하나만으로
실시간 위치나 정지를 대신하지 않는다. `ros2 topic info /cmd_vel -v`에서 구동 발행자는
trial_guard 하나인지 확인한다.

오류/HOLD 후에는 실제 정지·물품·로봇 위치를 확인하고 관제 복구 절차를 따른다.
runner/guard가 잠겼다면 담당 절차로 다시 시작하되 v2 journal은 보존한다.
구형 v1 journal은 최초 v2 설치 시에만 별도 파일로 분리한다. Ctrl+C와 프로세스 종료만으로
물리 정지 성공을 단정하지 않는다.
