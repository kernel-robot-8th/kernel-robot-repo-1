# pinky_test_console

Pinky Pro(ROS 2 Jazzy, Ubuntu 24.04, Raspberry Pi 5) 테스트용 웹 콘솔.

1. SLAM(slam_toolbox)으로 강의실 지도를 만들고 `~/roport_maps/`에 저장
2. Nav2·카메라를 띄웠을 때 메모리·CPU 상태 확인
3. IR 라인 센서 원시값을 보고 카펫·테이프 위에서 기록(`mode:=ir`, [IR 측정](#ir-측정-modeir))
4. 인쇄한 ArUco 마커가 카메라에서 제대로 인식되는지 확인(`mode:=marker`, [마커 테스트](#마커-테스트-modemarker))

원본 Pinky Pro 웹 서버(`pinky_navigation/scripts/nav2_web_server.py`, `index.html`, Apache-2.0)를 바탕으로 했습니다.
원본 출처와 수정 내용은 각 파일 상단에 적었습니다. 라이선스 전문은 [LICENSE](LICENSE)에 있습니다.

---

> ## ⚠️ 경고: 강의실 Wi-Fi 안에서만 사용
>
> - 이 콘솔은 **로그인 없이 로봇을 움직입니다.** 같은 네트워크에서 `http://<로봇IP>:8080`에 접속한 사람은 누구나 조종할 수 있습니다.
> - **강의실 Wi-Fi(또는 로봇 AP) 안에서만** 사용하세요. 공용망·인터넷에 노출하거나 포트포워딩하지 마세요. 암호화(HTTPS)도 없습니다.
> - 쓰지 않을 때는 콘솔을 끄세요(Ctrl+C). 끌 때 속도 0을 한 번 발행합니다.
> - 로봇은 바닥에 두고 주변을 비운 뒤 움직이세요. 화면 상단의 **■ 정지** 버튼(단축키 Space / Esc)을 항상 쓸 수 있게 두세요.
> - `pinky_bringup`에는 cmd_vel 타임아웃이 **없습니다.** 마지막 속도 명령을 계속 유지합니다.
>   콘솔은 정상 종료·예외·입력 끊김(0.3 s) 때 속도 0을 보냅니다.
>   하지만 프로세스가 `kill -9`나 OOM killer로 **강제 종료되면 0을 보낼 수 없습니다.**
>   메모리 한계 시험 중에는 로봇을 들어 올려 두거나, 바퀴가 바닥에 닿지 않게 하세요.

---

## 기능

| 기능 | 내용 |
|---|---|
| 지도·위치 | `/map`, Nav2 `plan`, global/local costmap, TF `map→base_link` 표시 (원본 기능) |
| SLAM | 지도 초기화(`/slam_toolbox/reset`), 이름을 받아 `~/roport_maps/<이름>.pgm/.yaml`로 저장(SaveMap에 절대경로 전달), 저장된 지도 목록 |
| Nav2 | 지도 클릭+드래그로 목표/초기 위치 지정, 목표 취소 (원본 기능) |
| 수동 조종 | 화면 방향 버튼(누르고 있는 동안) + 키보드 WASD·방향키, 속도 비율 슬라이더 |
| 정지 | 상단 큰 버튼: 속도 0 발행 + Nav2 목표 취소 |
| 상태 패널(1 s) | MemAvailable·사용량·스왑, loadavg, CPU 온도, 배터리 %/V, **콘솔 프로세스 RSS**, 기준 대비 메모리 증가량 |
| 마커 테스트(`marker.html`) | 카메라 영상에 인식한 마커 표시, ID·이름(`docs/aruco/markers.yaml`)·화면 크기·참고 거리, 표의 마커별 확인 ✓·최근 5 s 인식률 |
| IR 측정(`ir.html`) | `ir_sensor/range` 원시값 3개 실시간 표시·10 s 그래프, 라벨·시간 지정 CSV 기록(`~/roport_ir/`), 평균·표준편차, carpet/tape 참고 임계값 |

### 조종 안전 규칙 (서버에서 강제)

| 항목 | 값 | 구현 |
|---|---|---|
| 발행 토픽 | `/cmd_vel` `geometry_msgs/Twist` | `linear.x`[m/s], `angular.z`[rad/s] 외 0 |
| 발행 주기 | 10 Hz (조종 입력이 있는 동안만) | 입력이 없으면 아무것도 발행하지 않음 → Nav2 주행을 가리지 않음 |
| 속도 상한 | 선속도 ±0.10 m/s, 각속도 ±0.6 rad/s | 서버가 자름. NaN/inf/문자열/bool → 0 또는 400 |
| deadman | 0.3 s 동안 웹 입력 없으면 즉시 0 | 20 ms watchdog(STEADY 시계) → 만료 후 ≤20 ms 안에 0, 이어서 0을 3번 더 발행 |
| 손 뗌·창 전환 | 즉시 정지 요청 | `pointerup`/`keyup`/`blur`/`visibilitychange` |
| Nav2 충돌 | 주행 중 조종 입력 → **목표 취소 후** 조종 | Nav2 목표를 보내면 조종은 멈춤 |
| 종료·예외 | 속도 0 한 번 발행 | SIGINT/SIGTERM/SIGHUP(SSH 끊김), ROS 콜백 예외, Flask 종료 시 `finally` |

---

## 설치 의존성 (apt)

로봇(Raspberry Pi 5)과 노트북 공통:

```bash
sudo apt update
sudo apt install -y \
  python3-flask python3-pytest python3-opencv python3-yaml \
  ros-jazzy-slam-toolbox ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-map-server
```

- `pinky_navigation`(원본 launch·params)이 든 Pinky Pro 워크스페이스가 빌드되어 있어야 합니다.
  `mode:=slam`은 `map_building.launch.xml`, `mode:=nav`는 `bringup_launch.xml`을 그대로 include합니다.
- 노트북에서 Gazebo로 시험하려면 다음도 설치합니다: `ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher`.
  (`pinky_gz_sim`, `pinky_description`의 package.xml 기준)
- 브라우저 쪽은 설치할 것이 없습니다. 외부 CDN 없이 순수 HTML/JS입니다.

## 빌드

저장소의 `services/roport-pinky`에서 빌드합니다.

```bash
cd services/roport-pinky
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to pinky_test_console
source install/setup.bash
```

같은 워크스페이스의 원본 `src/pinky_pro/pinky_navigation`(launch·params)도 함께 빌드해야 하므로 `--packages-up-to`를 씁니다.
`--packages-select pinky_test_console`만 쓰면 "pinky_navigation이 빌드되지 않았다"는 오류로 실패합니다.

로봇처럼 `pinky_navigation`이 다른 워크스페이스(예: `~/pinky_pro`)에 이미 빌드돼 있으면, 그 워크스페이스를 먼저 source하고
이 패키지만 둔 워크스페이스에서 `colcon build --packages-select pinky_test_console`로 빌드해도 됩니다.

## 실행 순서 (실로봇)

```bash
# SSH 가 끊겨도 프로세스가 살아 있도록 tmux 안에서 실행 권장 (sudo apt install tmux; tmux new -s pinky)
# 터미널 1 (로봇) — 모터·라이다. 이 콘솔 launch 에는 포함하지 않음
ros2 launch pinky_bringup bringup_robot.launch.xml

# 터미널 2 (로봇) — 지도 만들기
ros2 launch pinky_test_console console.launch.py mode:=slam
```

브라우저에서 `http://<로봇IP>:8080`을 엽니다(로봇 IP: `hostname -I`).

1. **지도 만들기**: 조종 패드나 WASD로 강의실을 천천히 한 바퀴 돕니다. 회전은 특히 천천히 합니다.
   - 키보드는 **브라우저에서** WASD·방향키를 씁니다. 먼저 지도 영역을 한 번 클릭해 포커스를 주세요(지도 이름 입력칸에 커서가 있으면 키가 무시됩니다).
   - 터미널의 `teleop_twist_keyboard`도 쓸 수 있지만 주의할 점이 있습니다.
     기본 속도가 0.5 m/s / 1.0 rad/s이고 콘솔의 속도 제한·deadman을 거치지 않습니다.
     키를 한 번 누르면 명령을 한 번만 보내고, bringup은 그 명령을 계속 유지합니다(멈추려면 `k`).
     꼭 쓰려면 다음처럼 속도를 낮추세요. 웹 조종과 동시에 쓰면 안 됩니다.
     `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.1 -p turn:=0.5`
2. **저장**: SLAM 카드에 이름(예: `classroom`, 영문·숫자·`_`·`-`만)을 넣고 **Save Map**을 누릅니다.
   `~/roport_maps/classroom.pgm/.yaml`이 생기고 *Saved maps* 목록에 나옵니다.
   같은 이름이 있으면 덮어쓸지 묻습니다.
3. 콘솔을 Ctrl+C로 끄고 **nav 모드**로 다시 실행합니다(목록에서 지도를 누르면 명령이 표시됩니다).

```bash
ros2 launch pinky_test_console console.launch.py mode:=nav map:=$HOME/roport_maps/classroom.yaml
```

4. **Set Initial Pose**를 누르고 지도에서 로봇 위치를 클릭+드래그합니다(방향 = 드래그 방향). 그다음 **Set Goal** 상태에서 목표를 클릭+드래그합니다.
   - 로봇이 벽에 붙어 있으면(costmap 팽창 영역 안) Nav2가 경로를 만들지 못합니다(`Failed to create plan`). 먼저 조종으로 빈 곳으로 옮기세요.
   - Nav2 주행 속도는 원본 `nav2_params.yaml`이 정합니다(최대 0.25 m/s, 1.5 rad/s). 콘솔의 0.10 m/s / 0.6 rad/s 제한은 **수동 조종에만** 적용됩니다.

### launch 인자

| 인자 | 기본값 | 설명 |
|---|---|---|
| `mode` | `slam` | `slam` = slam_toolbox(원본 map_building과 동일 설정) + 콘솔, `nav` = 원본 bringup_launch(map_server+AMCL+Nav2) + 콘솔, `ir` = 콘솔 + IR 센서 노드만, `marker` = 콘솔 + 카메라 마커 인식만 |
| `map` | (없음) | `mode:=nav`일 때 필수. `.yaml` 경로(`~` 허용). 없거나 확장자가 틀리면 launch가 시작 전에 실패 |
| `port` | `8080` | 웹 포트(1–65535) |
| `ip` | `0.0.0.0` | 바인드 주소. 노트북 단독 시험이면 `127.0.0.1` 권장 |
| `use_sim_time` | `false` | Gazebo/MuJoCo 시뮬레이션이면 `true` |
| `map_dir` | `~/roport_maps` | 지도 저장 폴더 |
| `ir_dir` | `~/roport_ir` | IR 기록 CSV 폴더 |
| `camera` | `picamera2` | `mode:=marker` 카메라: `picamera2`(로봇) / `v4l2:<번호>`(USB 웹캠) / `topic:<이름>`(sensor_msgs/Image) |
| `rotate_180` | `auto` | 영상 180° 회전. `auto` = `picamera2`일 때만(이전 Pinky 코드와 같은 장착 방향) |
| `markers_yaml` | (없음) | 마커 ID 표 경로. 주면 이름·크기·확인 목록을 보여 줌 |
| `camera_hfov_deg` | `0` | 카메라 수평 화각(°). 주면 참고 거리 표시(보정 전 근사) |
| `ir_node` | `auto` | `pinky_sensor_adc main_node`를 함께 실행. `auto` = `mode:=ir`일 때만, `true`/`false`로 강제 |

잘못된 값(예: `mode:=explore`, `mode:=nav`인데 map 없음, `port:=99999`)은 노드를 하나도 띄우기 전에 오류로 끝납니다.

## IR 측정 (mode:=ir)

라인 인식 임계값을 정하기 위해 바닥 IR 센서(TCRT5000 × 3) 원시값을 기록합니다. **모터는 움직이지 않습니다**(bringup 불필요, 센서 노드는 ADC만 읽음).

```bash
# 로봇
ros2 launch pinky_test_console console.launch.py mode:=ir
# 이미 다른 터미널에서 main_node 를 띄웠다면: ... mode:=ir ir_node:=false
```

브라우저에서 `http://<로봇IP>:8080/ir.html`(콘솔 상단 **IR 측정** 버튼)을 엽니다.

1. **채널 확인**: 원본 노드는 `data = [adc2, adc1, adc0]` 순서로 보냅니다. 어느 `ir[n]`이 좌·중·우인지는 센서를 하나씩 가려 확인하고 기록해 둡니다.
2. 로봇을 손으로 옮겨 놓고 프리셋 라벨(`carpet`, `tape_center`, `tape_left`, `tape_right`, `tape_edge`)을 골라 5~30 s 기록합니다. 기록 중에는 로봇을 움직이지 않습니다.
3. `carpet*`와 `tape*` 기록이 모두 있으면 채널별 **참고 임계값**(두 평균의 중간)과 **분리도**(|차이| / 표준편차 합)를 보여 줍니다. 확정값이 아니라 판단 자료입니다.
4. CSV(`t_sec, ir0, ir1, ir2`)는 표의 ↓로 내려받거나 로봇의 `~/roport_ir/`에서 가져옵니다.

센서 간격(약 20 mm, Pinky Pro HW Spec)과 테이프 폭(20 mm)이 비슷해서, 가운데 센서가 테이프 중앙에 있으면 좌·우 센서는 테이프 밖에 놓입니다. `tape_edge` 기록으로 가장자리에서 값이 어떻게 변하는지도 확인하세요.

## 마커 테스트 (mode:=marker)

인쇄한 ArUco 마커가 로봇 카메라에서 맞는 ID로 인식되는지, 얼마나 먼 거리까지 보이는지 확인합니다. **모터는 움직이지 않습니다**(bringup 불필요).

```bash
# 노트북 → 로봇: ID 표 복사 (docs/aruco/markers.yaml, PR #11)
scp docs/aruco/markers.yaml pinky@<로봇IP>:~/markers.yaml
# 로봇
ros2 launch pinky_test_console console.launch.py mode:=marker markers_yaml:=~/markers.yaml
# 노트북 USB 웹캠으로 먼저 볼 때: ... mode:=marker camera:=v4l2:0 markers_yaml:=docs/aruco/markers.yaml
```

브라우저에서 `http://<로봇IP>:8080/marker.html`(콘솔 상단 **마커 테스트** 버튼)을 엽니다.

1. **ID 확인**: 마커를 한 장씩 카메라 앞에 대고 화면의 `ID 이름`이 종이 라벨과 같은지 봅니다. 오른쪽 목록에 ✓가 생깁니다(초록 = 표에 있는 ID, 빨강 = 표에 없는 ID).
2. **방향**: 보라 점이 마커의 왼쪽 위 모서리입니다. 종이의 위아래와 맞는지 확인합니다.
3. **거리**: 시험 시트(ID 0)를 줄자로 거리를 바꿔 대고, 인식률이 떨어지기 시작하는 거리를 정면·45°에서 기록합니다. 화면의 추정 거리는 `camera_hfov_deg`를 준 경우의 핀홀 근사(보정 전)라 줄자 값을 기준으로 합니다.
4. 결과는 Confluence 「ArUco 마커 ID 표 및 인쇄본 v1」의 시험 기록표나 Jira FFT1-74에 남깁니다.

- 로봇 카메라는 Picamera2로 읽습니다(`python3 -c "import picamera2"`가 되어야 함). 다른 프로그램이 카메라를 쓰고 있으면 열리지 않고, 화면 위에 오류가 표시됩니다.
- Pinky 이미지에는 libcamera가 두 벌 있습니다(`/usr/local`의 0.3, ROS `ros-jazzy-libcamera`의 0.7). ROS 환경을 source하면 `libpisp`만 ROS 쪽이 잡혀 `ipa_rpi_pisp.so: undefined symbol`로 죽기 때문에, launch가 `mode:=marker camera:=picamera2`일 때 `LD_LIBRARY_PATH` 앞에 `/usr/local/lib/aarch64-linux-gnu`를 붙여 실행합니다.
- 영상은 640×480, 인식은 콘솔 프로세스 안에서 합니다(OpenCV `cv2.aruco`, Ubuntu 24.04 apt 4.6 호환). 화면 색이 바뀌어 보여도 인식(흑백)에는 영향이 없습니다.

## 메모리 측정 방법

1. `bringup_robot` + 콘솔(`mode:=slam` 또는 `mode:=nav`)을 띄운 상태에서 상태 패널의 **기준 저장**을 누릅니다.
2. 다른 터미널에서 측정할 것(카메라 노드 등)을 실행합니다.
3. **Δ used since 기준**이 늘어난 사용량입니다. **MemAvailable**이 실제로 남은 여유입니다.
4. **Console RSS**는 이 웹 콘솔 프로세스만의 메모리입니다. 측정 대상에서 빼고 보세요. 괄호 안은 최대값(VmHWM)입니다.
   콘솔을 띄운 `ros2 launch` 프로세스도 메모리를 따로 씁니다. 프로세스별로 보려면 다음을 실행하세요.
   `ps -eo rss,pid,args --sort=-rss | head -15` (rss 단위 kB)
5. 지도·costmap 데이터는 바뀐 경우에만 전송됩니다. 그래도 costmap이 크면 콘솔 CPU 사용이 늘어납니다. Layers에서 costmap을 끄면 전송도 멈춥니다.

사용량 = `MemTotal − MemAvailable` (`free`의 used와 같은 기준). CPU 온도는 `/sys/class/thermal/thermal_zone0/temp`에서 읽습니다. 배터리는 `battery/percent`·`battery/voltage`(`std_msgs/Float32`, 5 s 주기) 토픽 값이고, 15 s 넘게 갱신이 없으면 "(오래됨)"으로 표시합니다.

## 노트북에서 Gazebo로 먼저 확인하기

실로봇 전에 조종·SLAM·저장을 시뮬레이션으로 확인할 수 있습니다. 노트북에 Pinky Pro 워크스페이스가 빌드돼 있어야 합니다(`pinky_description`, `pinky_gz_sim`, `pinky_navigation`, `pinky_bringup`).

```bash
# 터미널 1 — Gazebo (원본 README의 시뮬레이션 실행)
ros2 launch pinky_bringup bringup_sim.launch.xml sim:=gz
#   pinky_mujoco 를 빌드하지 않았다면 대신:
#   ros2 launch pinky_gz_sim launch_sim.launch.xml

# 터미널 2 — 콘솔 (sim 시간 사용, 노트북에서만 열기)
ros2 launch pinky_test_console console.launch.py mode:=slam use_sim_time:=true ip:=127.0.0.1
```

브라우저에서 `http://127.0.0.1:8080`을 열고 다음을 확인합니다.

1. WASD로 움직였다 떼면 로봇이 멈추는지
2. 지도가 그려지는지
3. Save Map 후 `~/roport_maps/`에 파일이 생기는지
4. 그 지도로 `mode:=nav map:=... use_sim_time:=true`가 뜨는지

시뮬레이션에는 배터리 토픽이 없어 "수신 없음"으로 표시됩니다. 메모리 수치는 노트북 값이라 로봇과 다릅니다.

## 로봇으로 옮기기

로봇의 `~/pinky_pro`에 원본 Pinky Pro(`pinky_navigation` 포함)가 이미 빌드돼 있다는 전제입니다.
노트북의 저장소 루트에서(로봇 사용자·IP는 환경에 맞게):

```bash
# 소스만 복사 (build/install/log 제외) — rsync 권장
ssh pinky@<ROBOT_IP> 'mkdir -p ~/pinky_test_ws/src'
rsync -av --delete services/roport-pinky/src/pinky_test_console/ pinky@<ROBOT_IP>:~/pinky_test_ws/src/pinky_test_console/

# rsync 가 없으면 scp
scp -r services/roport-pinky/src/pinky_test_console pinky@<ROBOT_IP>:~/pinky_test_ws/src/
```

로봇에서 빌드합니다. 로봇이 x86용 빌드 결과를 쓰지 않도록 `build/`·`install/`은 복사하지 않습니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/pinky_pro/install/setup.bash
cd ~/pinky_test_ws && colcon build --packages-select pinky_test_console
source ~/pinky_test_ws/install/setup.bash   # 새 터미널마다 (또는 ~/.bashrc 에 추가)
```

저장한 지도를 노트북으로 가져오려면 다음을 실행합니다.

```bash
rsync -av pinky@<ROBOT_IP>:~/roport_maps/ ~/roport_maps/
```

## 테스트 (로봇 없이)

```bash
cd services/roport-pinky
colcon build --packages-up-to pinky_test_console
colcon test --packages-select pinky_test_console --event-handlers console_direct+
colcon test-result --verbose
# 또는: source install/setup.bash && python3 -m pytest src/pinky_test_console/test -v
```

`python3-flask`가 없으면 Flask가 필요한 테스트 32개는 건너뜁니다(나머지 145개는 실행).

| 파일 | 검증 내용 |
|---|---|
| `test_teleop.py` | 속도 상한·NaN/문자열 처리, 0.3 s deadman, 정지 후 0 반복, 10 Hz+watchdog 타임라인 |
| `test_launch_plan.py` | `mode`/`map`/`port`/`use_sim_time`/`ir_node` 검사, include 대상(map_building / bringup_launch / ir은 없음), bringup_robot 미포함 |
| `test_web_app.py` | JSON 강제, 잘못된 입력 시 정지, 정적 파일 화이트리스트, 보안 헤더, 지도 이름 주입 차단, Nav2 취소 순서, IR API·CSV 다운로드 경로 검사 |
| `test_marker_detect.py` | 카메라 입력 지정 검사, ID 표 읽기, 합성 영상 인식·크기, 참고 거리, 확인 목록·인식률, ROS 이미지 변환 |
| `test_ir_record.py` | IR 최근값·주기, 시간 지정 기록·CSV, 센서 없음 처리, 라벨·파일 이름 검사, 참고 임계값 |
| `test_map_store.py` | 지도 이름·폴더 검사(쉘 메타문자·개행 거부), 목록 |
| `test_sysstats.py` | `/proc/meminfo`·loadavg·온도·RSS 파싱 |
| `test_node_ros.py` | 실제 rclpy로 노드를 띄워 `/cmd_vel` 10 Hz·상한·deadman·정지 확인 |

`test_node_ros.py`는 **ROS_DOMAIN_ID 199 + 전용 namespace**(`/pinky_console_pytest/cmd_vel`)로 발행합니다. 로봇 위에서 bringup이 켜진 채 실행해도 실제 `/cmd_vel`에는 발행되지 않습니다.

## HTTP API

| 메서드 | 경로 | 설명 |
|---|---|---|
| GET | `/api/info` | mode, 속도 상한, deadman, 지도 폴더 |
| GET | `/api/state?layers=map,global,local&map_v=..` | pose, path, Nav2/teleop 상태. grid는 버전이 바뀐 경우에만 데이터 포함 |
| GET | `/api/status` | 메모리·부하·온도·배터리·콘솔 RSS |
| GET | `/api/maps` | 저장된 지도 목록 |
| POST | `/api/teleop` `{linear, angular}` | 조종 입력 (0.3 s 안에 반복해야 유지) |
| POST | `/api/teleop/stop` | 조종 정지 |
| POST | `/api/estop` | 속도 0 + Nav2 취소 |
| POST | `/api/goal`, `/api/initialpose` `{x, y, yaw}` | Nav2 목표 / AMCL 초기 위치 (map frame, m, rad) |
| POST | `/api/nav/stop` | Nav2 목표 취소 |
| POST | `/api/slam/reset` | SLAM 초기화 |
| POST | `/api/slam/save_map` `{name, overwrite}` | 지도 저장 (이미 있으면 409) |
| GET | `/api/marker`, `/api/marker/frame.jpg` | 인식 결과·확인 목록 / 표시를 그린 최신 영상(없으면 204) |
| POST | `/api/marker/reset` | 확인 ✓ 기록 지우기 |
| GET | `/api/ir` | IR 최근값·주기·최근 10 s 기록·기록 진행 상태 |
| POST | `/api/ir/record` `{label, duration}` / `/api/ir/cancel` | IR 기록 시작(1–60 s) / 취소 |
| GET | `/api/ir/records`, `/api/ir/records/<파일>` | 기록 목록·통계·참고 임계값 / CSV 다운로드 |

POST는 `Content-Type: application/json`만 받습니다. 다른 웹사이트가 숨은 form으로 로봇을 움직이는 CSRF를 막기 위해서입니다.

## 알려진 제한

- **인증·HTTPS 없음**(위 경고). Flask 개발 서버를 그대로 씁니다(원본과 동일).
- `slam_toolbox`의 SaveMap은 받은 경로를 쉘 명령(`map_saver_cli -f …`)으로 실행합니다. 그래서 지도 이름은 `A-Z a-z 0-9 _ -`만, 저장 폴더 경로도 안전한 문자만 허용합니다.
- 조종 입력으로 Nav2 목표를 취소한 직후에는 Nav2 velocity_smoother가 감속 명령을 잠시 더 `/cmd_vel`에 보냅니다.
  Gazebo에서 잰 값은 취소 후 0.32 s 동안 −1.0 → 0 rad/s로 줄어드는 명령 7개였습니다. 그 뒤로는 조종 명령만 나갑니다.
- 종료 로그 `Published zero velocity on shutdown (subscribers=N, acked=…)`에서 N은 `/cmd_vel` 구독자 수(로봇이면 bringup 포함)입니다.
  `acked=True`면 모든 구독자가 0을 받았다는 확인입니다. 죽은 구독자가 디스커버리에 남아 있으면 `False`가 나올 수 있습니다.
- 콘솔 프로세스 RSS는 x86 노트북 측정값으로 약 100 MB입니다(Python + rclpy + Flask). Pi에서는 직접 확인하세요.
- local costmap은 odom 프레임이지만 원본과 같이 map 좌표로 그립니다. SLAM 드리프트가 크면 조금 어긋나 보일 수 있습니다.
- `mode:=slam`은 요구사항대로 slam_toolbox만 띄웁니다(원본 `web_slam`은 Nav2도 함께 띄움). 그래서 이 모드에서는 Nav2 목표를 쓸 수 없습니다.
