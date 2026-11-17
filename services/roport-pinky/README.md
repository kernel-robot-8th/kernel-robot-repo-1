# roport-pinky

RO:PORT · GATE CREW — Pinky 주행·도킹·로봇 명령 실행

## 기술 스택

Ubuntu 24.04 / ROS 2 Jazzy / Python 3.12 / colcon / ament_python. Nav2 연동은 실장비 주행 방식 합의 후 추가합니다.

## 현재 범위

개발 시작용 구조와 CI를 제공합니다. 로봇 제어·업무 기능·모델 추론은 아직 구현하지 않았습니다.
CI 성공은 실로봇·GPU·통합 검증 완료를 의미하지 않습니다.
배포 환경 미정으로 CD와 자동 배포는 설정하지 않았습니다.

## 관련 문서

이 디렉토리는 단일 저장소의 구성요소입니다. 아래 명령은 이 디렉토리에서 실행합니다.

- [프로젝트 개요](https://github.com/kernel-robot-8th/kernel-robot-repo-1)
- [요구사항·설계 문서](../../docs/README.md)
- [Jira](https://fp-team1.atlassian.net/jira/projects/FFT1)

PR에 관련 Jira 키, 변경 내용, 검증 결과와 미검증 항목을 기록합니다.

## 현장 G1 단독 왕복 시험 · FFT1-150

`scripts/field_roundtrip/`는 관제와 분리한 운영자 감독용 시험 실행부입니다.
기존 bringup·Nav2·ADC가 실행 중인 상태에서 UI의 출발 버튼으로
H1→N12→G1을 진행하며, 전달 완료 버튼을 누르기 전에는 복귀하지 않습니다.
도착 시 기존 Picamera2 발행자가 없으면 카메라를 시작합니다.
마커 거리 보정은 0.01m/s, 1회 최대 1cm·총 6회로 제한하고,
영상·위치·독점 구동권 또는 충돌 검사가 실패하면 정지합니다.
마커 확인 실패는 정밀 주차 성공으로 취급하지 않으며, 정지 확인 후
경고를 표시하고 운영자의 복귀 명령을 기다립니다. 각도 정밀 보정은 미구현입니다.

복귀는 Nav2로 N12에 진입한 뒤 실제 IR 중심 감지를 0.5초 확인하고
N12→N08→N09→H1 라인트레이싱으로 전환합니다. 중앙 IR이 맞지 않으면 먼저
실측 N12 목표와 현재 위치의 차이가 8cm 이내일 때만 Nav2의 충돌 검사된
Spin→DriveOnHeading→Spin으로 한 번 보정한 뒤, 필요할 때만 6cm 선 탐색을 합니다. 선을 찾지 못하면
정지하며 IR 값·측정 좌표·보정 검증 플래그를 덮어쓰지 않습니다.
이 시험 launch의 Nav2 상한은 0.12m/s, 라인 실행부는 0.06m/s입니다.
bringup·모터 초기화·목표 전송은 launch나 웹 서버 기동만으로 실행하지 않습니다.

N12 복귀 목표는 고정 `return_n12`를 사용하지 않습니다. **이번 출발의 실제
라인→Nav2 전환점**에서 정지·위치 추정·N12 handoff 상태·새 IR 가로선 감지를
0.5초 확인한 뒤 기록합니다. IR 센서와 바퀴 축 사이 거리(`ir_ahead_cm`) 및
방향 반전을 반영해 복귀 중심 좌표를 계산하고, 실제로 통과한 공항 진입점도
복귀 경유점으로 저장합니다. 지도상 N12 근처 8cm·선 방향 0.1rad 범위를
벗어난 가로선은 N12로 학습하지 않습니다.

관측은 현장 폴더 `n12-handoffs/<왕복ID>.json`, 해당 실행의 파생 Nav2 목표는
`runtime/<실행ID>-places.yaml`에 저장합니다. Nav2 목표 전송·N12 자세 보정·최종
IR 획득의 위치 기준이 모두 이 기록을 사용합니다. 원본 `field-places.yaml`의
실측값과 지도 해시·검증 플래그는 변경하지 않습니다. 재시도에서는 원래 왕복ID,
기체, 부팅ID, 지도 버전, layout·원본 목적지 파일 해시, IR 오프셋이 같은 기록만
사용합니다. 기록이 없거나 설정이 바뀌면 고정 좌표로 추측하지 않고 정지합니다.
새 배포 시 `roundtrip.py`와 **`mission_handoff.py`를 함께** 복사해야 합니다.
좌표 재사용은 AMCL의 왕복 중 오차 변화나 좁은 곳의 회전 충돌까지 해결한다는
뜻이 아니며, 실제 복귀에서도 신선한 IR·정지·구동권·충돌 검사를 유지합니다.

마커 검출은 동일 프레임·보정 파일에 대해 한 번만 수행하고, 검출기와 마커 형상은
재사용합니다. 캐시 조회 전 영상 시각을 다시 검사하므로 오래된 결과가 새 관측이 되지
않으며, 보정 파일이 바뀌면 같은 프레임도 재처리합니다. 구동 가드 JSON 검사는
`roport_smartcart.trial_contract.guard_body`를 공유하며 잘못되거나 지연된 패킷은
정상 상태를 갱신하지 않습니다. 실행 스크립트와 스마트카트 패키지를 함께 빌드·배포해야 합니다.

운영자가 검증한 지도와 `layout.yaml`, `field-places.yaml`, `h1-confirmed.json`,
`camera_calibration.yaml`, `mission-target.json`(`"G1"`)을 별도 현장 폴더에 둡니다.
거리 보정에는 그 지도·보정 파일에 대응하는 `gate-distance-reference.json`의
`marker_id`, `map_version`, `calibration_sha256`, `front_minus_lens_m`도 필요합니다.
마지막 값은 같은 정지 자세에서 실측한 앞면-벽 거리에서 영상의 렌즈-평면 거리를
뺀 값(m)이며 ±0.03m 이내만 허용합니다. 한 지점의 거리 기준은 카메라 광학 TF나
모든 거리의 정확도 검증을 대신하지 않습니다. 이 현장 파일·사진은 커밋하지 않습니다.

시험 파일은 현장 폴더로 복사하되 기존 실측 파일을 덮어쓰지 않습니다.
ROS overlay를 source하는 별도 setup 파일을 준비합니다. PC의 기존 UI는 아래처럼
실행하며 SSH는 사전에 인증된 키와 known_hosts를 사용합니다. 비밀번호를 코드에 넣지 않습니다.

```bash
export ROPORT_FIELD_CONFIG_DIR=/absolute/path/to/local-field-config
export ROPORT_FIELD_REMOTE_DIR=/absolute/path/to/robot-field-config
export ROPORT_FIELD_REMOTE_SETUP=/absolute/path/to/robot-ros-setup.sh
export ROPORT_PINKY_SSH_HOST=robot@example.com
export ROPORT_PINKY_CONSOLE_URL=http://example.com:8080
source /opt/ros/jazzy/setup.bash
# 이어서 PC의 검증된 ROS overlay를 source합니다.
export ROS_DOMAIN_ID=13
python3 scripts/field_roundtrip/ui_server.py
```

UI는 `http://127.0.0.1:8086`에만 열립니다. 로봇 쪽 `roundtrip.py`는 동일한
`ROPORT_FIELD_CONFIG_DIR`, `ROPORT_PINKY_MAP`, `ROBOT_NAME`, domain13 환경에서
`--drive`가 있어야 실행됩니다. `--resume-delivery`는 실측 게이트에 정지한 경우
접근 동작 없이 카메라 확인·버튼 대기만 재개합니다. `--resume-n12`는 직전 정지
기록과 현재 자세가 5cm·0.1rad 이내로 일치할 때만 N12 보정·IR 복귀를 재시도합니다.
전원을 끄거나 손으로 옮긴 후에는 이전 체크포인트로 재개하지 않습니다. N12 오류로
종료된 경우 기존 복귀 버튼은 **N12 복귀 재시도 → H1**으로 바뀝니다. 현재의 신선한
정지 자세가 같은 시험의 실제 정지 기록과 일치해야 하며, 재부팅 이전 기록은 거부합니다.
선 탐색으로 이미 지나친 경우에만 기록된 탐색 시작점까지 7cm 이내의 직선 후진을
허용하고, 횡방향 차이 1.5cm·방향 차이 0.1rad를 벗어나면 재개하지 않습니다.
후진에도 Nav2 충돌 검사·구동권·센서/위치 검사를 적용하며 보정 범위 8cm는 유지합니다.

UI의 **브링업·Nav2·센서 다시 켜기** 버튼은 신선한 정지 관측과 구동권 해제,
진행 중인 시험이 없을 때만 사용할 수 있습니다. 로봇의 현장 폴더에
`restart_stack.py`와 `stack-sessions.json`을 둡니다. JSON은 `bringup`, `adc`,
`nav2`, `console`을 해당 도구가 관리하는 `field_` 접두사의 실제 tmux 세션 이름에
대응시킵니다. 재시작은 로봇에서도 정지·활성 시험 유무를 다시 확인하며,
기존 세션이 종료되지 않으면 중복 노드를 띄우지 않습니다. 현재 관측된 지도 자세를
복원하고 Nav2·위치·정지 상태를 확인할 뿐 목표·속도·구동권 명령은 보내지 않습니다.
센서 연결이 끊겨 정지를 확인할 수 없으면 버튼으로 강제 재시작하지 않습니다.

2026-11-17 실물 v14 시험에서 G1 좌표 도착, 마커 거리 접근(19.8cm 실측 시작점에서
영상 추정 약 16.8cm까지), N12 자세 보정 후 H1 복귀·정지를 확인했습니다.
N12 재시도는 별도 운영자 명령으로 확인했으며, 자동 1회 보정으로 연결한 새 코드의
반복 실주행·main v15 실주행·다른 게이트·마커 기준 각도 보정은 미검증입니다.
관제 runner의 프로토콜·도착 상태나 `main` 지도는 이 시험 실행부가 변경하지 않습니다.
실로봇 적용 전 장비·설정·캘리브레이션, 안전 정지·복구 절차를 확인합니다.

## 개발 방법

Ubuntu 24.04에 ROS 2 Jazzy와 colcon을 설치한 뒤 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select roport_pinky
source install/setup.bash
python3 -c "import roport_pinky; import rclpy"
```

팀 패키지 중 실행 노드는 IR 라인 추종 노드 v0([src/roport_pinky_line/](src/roport_pinky_line/), `cmd_vel` 발행, 실로봇 1~3단계 시험 2026-11-03)와 노드 경로 주행([src/roport_line_nav/](src/roport_line_nav/), 물류·스마트카트 공용, 실로봇 미검증)입니다. 스마트카트 작업 위치는 [src/roport_smartcart/](src/roport_smartcart/)입니다. 아래 원본 Pinky Pro에는 실장비 실행 코드가 포함돼 있습니다.

`roport_smartcart`의 작업 계획과 오프라인 검증 구성은 [src/roport_smartcart/](src/roport_smartcart/README.md)에 있습니다. 자동 주행 실행 노드는 아직 없습니다. 아래 원본 Pinky Pro에는 실장비 실행 코드가 포함돼 있습니다.

## 시연 공간 Gazebo world

[src/roport_gz_world/](src/roport_gz_world/README.md)는 시연 공간 실측 배치(v14)를 Gazebo world로 만든 패키지입니다.
world 좌표를 실제 SLAM 지도 좌표계에 맞춰 두어 시뮬레이션 Nav2에서 실제 지도를 그대로 씁니다.
`colcon build --packages-up-to roport_gz_world` 후 `ros2 launch roport_gz_world roport_gz_world.launch.xml`로 실행합니다.
가정값(`provisional`)은 패키지 README에 정리돼 있습니다.

## Pinky Pro 원본

[src/pinky_pro/](src/pinky_pro/)에 공식 소스를 포함했습니다. 커밋과 라이선스는 [src/UPSTREAM.md](src/UPSTREAM.md)에 기록합니다. 원본은 Ubuntu 24.04 / ROS 2 Jazzy를 안내합니다.

원본 전체를 빌드하려면 의존성을 준비한 ROS 환경에서 실행합니다. 설치 명령은 시스템 패키지를 추가하며, 빌드 자체는 모터를 구동하지 않습니다.

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

기본 CI 실행: `bash ci/test.sh` — 팀 패키지·원본 인터페이스·description 빌드, URDF 전개와 수신 검사 로직을 검증합니다.

실로봇 bringup이 이미 실행 중인 상태에서 같은 `ROS_DOMAIN_ID`로 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/check_hardware.py --namespace / --output /tmp/pinky-check.json
```

`odom`·`joint_states`·`scan`에서 timestamp가 증가하는 메시지를 각각 3개 수신합니다. 명령을 발행하거나 bringup을 실행하지 않습니다. 실패 시 exit code 1과 JSON 결과를 남깁니다. 수신 성공만으로 실로봇/시뮬레이터 구별이나 동작 안전성을 판정하지 않습니다. ARM64 드라이버·전체 시뮬레이션·실주행은 별도 검증이 필요합니다.

## 테스트 웹 콘솔

[src/pinky_test_console/](src/pinky_test_console/README.md)은 SLAM 지도 제작과 Nav2·카메라 실행 시 메모리 확인을 위한 브라우저 콘솔입니다.
원본 Pinky Pro 웹 서버(Apache-2.0)를 바탕으로 웹 조종(0.10 m/s·0.6 rad/s 서버 제한, 0.3 s deadman, 정지 버튼)과 상태 패널을 더했습니다.
로그인 없이 로봇을 움직이므로 강의실 Wi-Fi 안에서만 사용합니다. 빌드는 `colcon build --packages-up-to pinky_test_console`입니다.

## 시연 공간 지도

[maps/](maps/)에 시연 공간 지도를 둡니다. 실로봇 Nav2·Gazebo 시뮬레이션·관제 GUI가 같은 파일을 씁니다.

| 파일 | 내용 |
|---|---|
| `airport_map.pgm`, `airport_map.yaml` | Nav2용 지도. 실측 배치 v15(2026-11-02 실측, 02 인도장 2026-11-06·가로선 2026-11-16 갱신)를 그린 지도이며 SLAM 지도가 아닙니다. 벽과 라이다(12.5 cm)보다 높은 물체만 점유로 그립니다. 92×53칸, 0.05 m/칸, origin (−1.654, −2.094) m, `map_server` 형식 |
| `airport_map.png` | `airport_map.pgm`과 같은 픽셀(0 점유, 205 미지, 254 빈 곳)의 PNG. 브라우저 등 pgm을 못 읽는 곳에서 사용. 행 0이 지도의 위쪽(+y 끝)입니다 |
| `airport_map_slam_20261027.*` | 2026-10-27 실로봇(Pinky Pro)으로 slam_toolbox를 돌려 만든 지도. 실측 벽 위치 검증용(03 구역 T자 구조물은 지금 없음) |
| `map_measured_v15.png`, `_02.png` | 실측 도면과 라인 노드 그래프(노드 좌표·연결 표 포함). 단위 cm, 원점 = 콘센트 벽·통로 쪽 모서리 |

- 원본은 [roport_gz_world/layout/roport_airport.yaml](src/roport_gz_world/layout/roport_airport.yaml)입니다. 벽·물체·노드를 여기서 고치고 지도·도면·Gazebo world를 다시 생성합니다([roport_gz_world README](src/roport_gz_world/README.md)).
- 지도 좌표(m)는 `map` 프레임입니다. 실측 좌표(cm)와의 변환은 같은 파일의 `frame`에 있습니다.
- 사진(사람이 찍힘)은 올리지 않습니다.
