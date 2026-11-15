# pinky_test_console

Pinky Pro(ROS 2 Jazzy, Ubuntu 24.04, Raspberry Pi 5) 테스트용 웹 콘솔.

1. SLAM(slam_toolbox)으로 강의실 지도를 만들고 `~/roport_maps/`에 저장
2. Nav2·카메라를 띄웠을 때 메모리·CPU 상태 확인
3. IR 라인 센서 원시값을 보고 카펫·테이프 위에서 기록(`mode:=ir`, [IR 측정](#ir-측정-modeir))
4. 인쇄한 ArUco 마커가 카메라에서 제대로 인식되는지 확인(`mode:=marker`, [마커 테스트](#마커-테스트-modemarker))
5. 실측 지도로 AMCL 위치가 맞는지 확인(`mode:=amcl`, [AMCL 테스트](#amcl-테스트-modeamcl))

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
| AMCL 테스트(`amcl.html`) | 지도 위 라이다 스캔(벽까지 5·10 cm로 색 구분)·AMCL 위치·표준편차, 정합 점수, 배치 yaml 노드(N12 등) 초기 위치 버튼, 제자리 갱신, 실제 위치와 비교 기록(CSV) |
| 마커 테스트(`marker.html`) | 카메라 영상에 인식한 마커 표시, ID·이름(`docs/aruco/markers.yaml`)·화면 크기·참고 거리, 표의 마커별 확인 ✓·최근 5 s 인식률 |
| IR 측정(`ir.html`) | `ir_sensor/range` 원시값 3개 실시간 표시·10 s 그래프, 라벨·시간 지정 CSV 기록(`~/roport_ir/`), 평균·표준편차, carpet/tape 참고 임계값 |
| 경로 주행(`route.html`, 로봇) | 노드 그래프에서 경로 지정(클릭·경로 예시), 시작·상차 완료·정지·**⌂ P5 복귀**, 배터리(%·V), `line_nav/state`(구간·감지 창·회전·IR·이벤트) 표시. 경로 주행은 [roport_line_nav](../roport_line_nav/) |
| 통합 테스트(`test.html`, 노트북) | 탭 하나로 테스트 관제·Pinky 설정과 로봇별 경로 주행·IR 측정·조종/지도·AMCL 화면을 엶(로봇 화면은 노트북이 `/robot/<이름>/`으로 중계). 마커 탭은 마커 부착·시험 뒤 추가 |
| 테스트 관제(`fleet.html`, 노트북) | 전체 지도(벽·물체·구역·게이트·ARR)에 로봇별 추정 위치·지나온 구간(로봇 색, 새 경로 시작 때 지움). 출발(P1·P2·P5·P6 또는 현재 위치) 고르고 목적지 정차 노드를 누르면 최단 경로. 시작·상차 완료·정지·복귀(경로 끝 상차 대기 중이면 로봇 콘솔이 상차 완료 뒤 출발), 전체 정지·전체 복귀. 로봇 표에 배터리 %. 로그 화면(로봇 ROS 로그 포함)·CSV 저장. 로봇 콘솔 원격 켜기·끄기(`test`·`ir`), 로봇 로그 파일 가져오기(scp). 각 로봇의 `mode:=test`(또는 `route`) 콘솔을 HTTP로 부름(MQTT 없음) |
| Pinky 설정(`settings.html`, 노트북) | IP로 등록(이름·역할·ROS_DOMAIN_ID·IR 기준값·복귀 칸·RMW), 연결 확인, 업데이트(브랜치)·릴리즈(`origin/main`) 설치, 빌드, 도메인 읽기·적용, 작업 기록 |

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
| `mode` | `slam` | `slam` = slam_toolbox(원본 map_building과 동일 설정) + 콘솔, `nav` = 원본 bringup_launch(map_server+AMCL+Nav2) + 콘솔, `ir` = 콘솔 + IR 센서 노드만, `marker` = 콘솔 + 카메라 마커 인식만, `amcl` = 원본 localization_launch(map_server+AMCL만, 경로 계획·주행 제어 없음) + 콘솔, `route` = 콘솔 + `roport_line_nav` 경로 주행 노드 + IR 센서 노드(로봇), `test` = `route`와 같은 노드로 경로 주행·IR 측정 화면을 함께 씀(로봇, 통합 화면용), `fleet` = Pinky 설정·테스트 관제(노트북, 주행·Nav2 노드는 띄우지 않음) |
| `map` | (없음) | `mode:=nav`·`amcl`일 때 필수. `.yaml` 경로(`~` 허용). 없거나 확장자가 틀리면 launch가 시작 전에 실패 |
| `port` | `8080` | 웹 포트(1–65535) |
| `ip` | `0.0.0.0` (`fleet`은 `127.0.0.1`) | 바인드 주소. 노트북 단독 시험이면 `127.0.0.1` 권장. `fleet`은 SSH 권한이 있어 기본이 이 PC 전용 |
| `robot` | (없음) | `mode:=route`·`test` 필수. IR 기준값 이름(`pinky_d660`·`pinky_6422`·`pinky_764e`, `roport_pinky_line/config/robots.yaml`) |
| `fleet_file` | `~/.roport/fleet.json` | `mode:=fleet` 등록 목록(권한 600, 비밀값 없음) |
| `repo_dir` | `~/dev_ws/roport/kernel-robot-repo-1` | `mode:=fleet`에서 `git archive`할 노트북 저장소 |
| `ssh_key` | `~/.ssh/pinky_ed25519` | `mode:=fleet` SSH 키. 비밀번호는 받지 않음(BatchMode) |
| `fleet_log_dir` | `~/roport_fleet_logs` | `mode:=fleet` 날짜별 로그 CSV 폴더. 로봇에서 가져온 파일은 `<폴더>/<로봇 이름>/` |
| `dock_dir` | `~/roport_dock` | `mode:=fleet` 정밀 도킹 탭의 `cameras.json`·`calib_*.json` 폴더(없으면 기본 카메라 2대) |
| `nav_log_dir` | `~/roport_logs` | `mode:=route`·`test` 경로 상세 기록(route_*.csv) 폴더(line_nav `log_dir`) |
| `line_max_linear` | `0.06` | `mode:=route`·`test` line_nav 직선 속도 상한(m/s, 코드 상한 0.15). 화면 속도 슬라이더 최대. 0.08 시험: `line_max_linear:=0.08` |
| `approach_lead_cm` | `5.0` | `mode:=route`·`test` 예상 노드 위치 몇 cm 전부터 0.03 m/s로 감속(line_nav) |
| `line_kp`·`line_kd` | `1.0`·`0.05` | `mode:=route`·`test` 라인 추종 게인(line_nav `kp`·`kd`) |
| `line_gain_ref_speed` | `0.06` | 이 속도(m/s)까지는 `kp` 그대로, 넘으면 `kp × 속도/이 값`(0.08이면 1.33). `0.15`면 속도 맞춤 끔. 시험판, 실로봇 미검증 |
| `use_sim_time` | `false` | Gazebo/MuJoCo 시뮬레이션이면 `true` |
| `map_dir` | `~/roport_maps` | 지도 저장 폴더 |
| `ir_dir` | `~/roport_ir` | IR 기록 CSV 폴더 |
| `camera` | `picamera2` | `mode:=marker` 카메라: `picamera2`(로봇) / `v4l2:<번호>`(USB 웹캠) / `topic:<이름>`(sensor_msgs/Image) |
| `rotate_180` | `auto` | 영상 180° 회전. `auto` = `picamera2`일 때만(이전 Pinky 코드와 같은 장착 방향) |
| `markers_yaml` | (없음) | 마커 ID 표 경로. 주면 이름·크기·확인 목록을 보여 줌 |
| `camera_hfov_deg` | `0` | 카메라 수평 화각(°). 주면 참고 거리 표시(보정 전 근사) |
| `layout` | (없음) | 배치 yaml(`roport_gz_world/layout/roport_airport.yaml`). `mode:=amcl`은 주면 노드 버튼과 실측 좌표(cm) 입력을 씀. `mode:=route`·`fleet`은 비우면 설치된 `roport_gz_world`의 파일 |
| `amcl_dir` | `~/roport_amcl` | AMCL 기록 CSV 폴더 |
| `ir_node` | `auto` | `pinky_sensor_adc main_node`를 함께 실행. `auto` = `mode:=ir`·`route`일 때만, `true`/`false`로 강제 |

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
4. **거리별 기록**: 오른쪽 「거리별 기록」에 로봇 최앞단~마커 거리(cm)를 넣고 「기록」을 누르면, 몇 초 동안 인식률·화면 크기(px)·마커 위아래 위치를 모아 표에 한 줄 남깁니다(`~/roport_marker/range_log.csv`, CSV ↓로 내려받기). 같은 거리에서 50 mm 마커였을 때의 예상 크기도 보여 줍니다.
5. 결과는 Confluence 「ArUco 마커 ID 표」의 시험 기록표나 Jira FFT1-108에 남깁니다.

- 로봇 카메라는 Picamera2로 읽습니다(`python3 -c "import picamera2"`가 되어야 함). 다른 프로그램이 카메라를 쓰고 있으면 열리지 않고, 화면 위에 오류가 표시됩니다.
- Pinky 이미지에는 libcamera가 두 벌 있습니다(`/usr/local`의 0.3, ROS `ros-jazzy-libcamera`의 0.7). ROS 환경을 source하면 `libpisp`만 ROS 쪽이 잡혀 `ipa_rpi_pisp.so: undefined symbol`로 죽기 때문에, launch가 `mode:=marker camera:=picamera2`일 때 `LD_LIBRARY_PATH` 앞에 `/usr/local/lib/aarch64-linux-gnu`를 붙여 실행합니다.
- 영상은 640×480, 인식은 콘솔 프로세스 안에서 합니다(OpenCV `cv2.aruco`, Ubuntu 24.04 apt 4.6 호환). 화면 색이 바뀌어 보여도 인식(흑백)에는 영향이 없습니다.

## AMCL 테스트 (mode:=amcl)

실측 지도(`services/roport-pinky/maps/airport_map.yaml`, SLAM이 아니라 실측 배치로 그린 지도)로 AMCL을 띄웠을 때 위치가 맞는지 확인합니다(Jira FFT1-123). Nav2는 N12(간선 끝) 이후 03 공용 내부공간에서만 쓰므로 **03 구역과 N12만** 확인합니다.
이 모드는 map_server와 AMCL만 띄웁니다. **경로 계획·주행 제어가 없어서 로봇이 스스로 움직이지 않습니다.** bringup(라이다·바퀴 odom)은 따로 먼저 띄웁니다.

```bash
# 노트북 → 로봇: 지도와 배치 yaml 복사 (PR #10)
scp services/roport-pinky/maps/airport_map.{yaml,pgm} \
    services/roport-pinky/src/roport_gz_world/layout/roport_airport.yaml pinky@<로봇IP>:~/roport_maps/
# 로봇
ros2 launch pinky_test_console console.launch.py mode:=amcl \
    map:=~/roport_maps/airport_map.yaml layout:=~/roport_maps/roport_airport.yaml
```

브라우저에서 `http://<로봇IP>:8080/amcl.html`(콘솔 상단 **AMCL 테스트** 버튼)을 엽니다.

1. **N12**: 로봇을 N12 가로선 위에 +x 방향(입구 쪽)으로 놓고 「N12 (Nav2)」 버튼 → 「제자리 갱신」을 3~5번 누릅니다. AMCL은 원래 0.15 m 또는 0.1 rad 움직여야 갱신하는데, 제자리 갱신(`request_nomotion_update`)은 로봇을 움직이지 않고 한 번 갱신합니다.
2. **점수**: 「벽 10 cm 안」이 80% 이상이면 합격 표시(제안 기준)입니다. 화면의 빨간 점이 한 벽에 몰려 있으면 그 벽 위치(배치 yaml)나 실측을 다시 봅니다.
3. **기록**: 줄자로 잰 실제 위치(cm, 실측 도면 좌표)를 넣고 「기록」을 누르면 AMCL 위치와의 오차, 표준편차, 점수가 `~/roport_amcl/amcl_<날짜>.csv`에 한 줄 남습니다.
4. **03 안**: 로봇을 손으로(또는 콘솔 조종 0.10 m/s로) 4~5곳에 옮기며 같은 순서로 기록합니다. 예: G1·G2·G3 앞(x 170), 상자 옆, 입구.
5. CSV를 Jira FFT1-123 댓글에 첨부합니다.

- 정합 점수 = 라이다 스캔 점(최대 360개) 중 지도 점유 칸에서 5·10 cm 안에 든 비율. 스캔을 받은 순간의 TF(map → 라이다)로 점을 옮기므로 로봇을 멈춘 상태에서 봅니다.
- 실측 좌표(cm) ↔ map(m) 변환은 배치 yaml의 `frame`을 씁니다. `layout:=`이 없으면 노드 버튼 없이 map 좌표(m × 100)로만 입력·기록합니다.
- 노드·실측 좌표로 주는 초기 위치는 표준편차 5 cm·5°로 보냅니다(콘솔 지도의 「Set Initial Pose」는 RViz 기본 0.5 m). 로봇을 노드에 정확히 놓는 시험이라 좁게 주며, 넓게 주면 제자리 갱신으로 수렴이 느립니다(Gazebo N12: 갱신 15번 뒤 12 cm).
- 초기 위치가 크게 틀리면 AMCL이 제자리에서 스스로 바로잡지 못합니다. 이때 점수가 떨어지므로(아래 18 cm 시험) 점수가 80% 아래면 로봇 위치·방향을 다시 맞추고 초기 위치를 다시 줍니다.

Gazebo 확인(v13 world·실측 지도, 로봇을 순간 이동한 뒤 초기 위치 → 제자리 갱신 3~5번, 2026-11-03):

| 지점 (실측 cm, 방향) | 위치 오차 | 방향 오차 | 벽 5 cm 안 | 벽 10 cm 안 |
|---|---|---|---|---|
| N12 (378.3, 73.2, 0°) | 2.1 cm | 0.2° | — | 99.7% |
| G1 앞 (170, 120.9, 180°) | 0.6 cm | 0.6° | 100% | 100% |
| 상자 옆 (300, 125, 90°) | 2.0 cm | 0.3° | 100% | 100% |
| 입구 (412, 75, 90°) | 3.4 cm | 0.4° | 99.4% | 100% |
| 초기 위치를 18 cm 틀리게 줌 | 17.2 cm | 4.9° | 52.8% | 78.1% (확인 필요) |
| 로봇만 N12로 옮기고 초기 위치를 안 줌 | — | — | 32.8% | 40.6% |

시뮬레이션 결과는 실로봇 시험을 대신하지 않습니다(유리·검은 벽·사람 등 실제 라이다 조건이 다름).

## 경로 주행 (mode:=test 또는 route, 로봇)

```bash
# 터미널 1 — 모터·odom
ros2 launch pinky_bringup bringup_robot.launch.xml
# 터미널 2 — 콘솔 + 경로 주행 노드 + IR 센서 노드
ros2 launch pinky_test_console console.launch.py mode:=test robot:=pinky_d660   # 경로 + IR 측정
```

터미널 2는 노트북 테스트 관제의 「경로+IR 켜기」로 대신할 수 있습니다(터미널 1 bringup은 원격으로 켜지 않음).

브라우저에서 `http://<로봇IP>:8080/route.html`을 엽니다(콘솔 상단 **경로 주행** 버튼).

- **⌂ P5 복귀**: 지금 노드에서 P5(N16)까지 최단 경로로 출발(`POST /api/route/return {start?, goal?}`).
  - 끝난 경로의 도착 노드(arrived·handoff)에서 바로 출발합니다.
  - 경로 끝 상차 대기(loading)면 상차 완료를 보내고 도착(arrived)을 확인한 뒤 출발합니다(최대 40 s). 그 사이 경로 정지·비상 정지·다른 경로 명령이 오면 복귀는 취소되고 출발하지 않습니다.
  - 대기·정지·오류 뒤나 상태가 오래되어 위치를 모르면 버튼 옆 **「지금 위치」**에서 로봇이 서 있는 노드를 직접 고릅니다(지난 경로의 출발점은 쓰지 않음, 한 번 쓰면 비움). 첫 구간 방향으로 세워 둡니다.
  - 주행 중·경유 상차 중에는 거부합니다. 로봇끼리 충돌 회피는 없습니다.
- 경로 예시 **P5 → P4 상차 → P5 왕복**: P4에서 상차 대기(「상차 완료」) 뒤 P5 칸으로 돌아와 후진 정차.
- 상단 **배터리** 칩: `battery/percent`·`voltage`(bringup의 battery_publisher, 5 s 주기). 40% 미만 노랑, 20% 미만 빨강, 15 s 넘게 안 오면 흐리게.

1. 로봇을 첫 노드 위에 첫 구간 방향으로 세웁니다(대기·충전 칸·상차 노드에서 끝난 경로는 그 자리에서 바로 다음 경로).
2. 지도에서 노드를 차례로 누르거나 「경로 예시」를 고르고 **경로 시작**. 예시에 「P5 → P4 상차 → P7 상차 (한 번에)」(P4 경유)·「P5 → P7 바로」가 있음. 「속도 적용」으로 직선·회전 속도를 바꿀 수 있음(최대 6 cm/s·0.6 rad/s).
3. N01·N11에서 반시계 90° 돌고 `loading`이 되면 상차 후 **상차 완료**를 누릅니다.
4. 상단 **■ 정지**(Space/Esc)는 경로 주행 정지 명령 + 속도 0을 보냅니다.

`roport_gz_world`·`roport_pinky_line`·`roport_line_nav`도 같은 워크스페이스에 빌드돼 있어야 합니다(Pinky 설정의 설치가 함께 보냄).
`line_follower`(라인 추종 v0)와 같이 실행하지 않습니다.

## Pinky 설정 · 테스트 관제 (mode:=fleet, 노트북)

관제 서버(roport-control-server) Pinky Settings 화면을 전달하기 전에 기능을 시험하는 도구입니다. MQTT는 쓰지 않습니다.

```bash
ros2 launch pinky_test_console console.launch.py mode:=fleet      # http://127.0.0.1:8080/ (= test.html 통합 화면)
```

| 작업 | 실행하는 것 |
|---|---|
| 연결 확인 | `ssh -i <ssh_key> -o BatchMode=yes pinky@<IP> 'echo ok; hostname'` |
| 업데이트 설치 / 릴리즈 설치 | `git fetch` → `git rev-parse --verify <ref>` → `git archive <ref> <패키지 5개> \| ssh pinky@<IP> 'tar xf - --strip-components=3 -C ~/pinky_test_ws/src'`. 릴리즈 = `origin/main` |
| 빌드 | 로봇에서 `colcon build --packages-select pinky_test_console roport_pinky_line roport_line_nav roport_gz_world roport_pinky` |
| 도메인 읽기 | `grep -n ROS_DOMAIN_ID ~/.bashrc` |
| 도메인 적용 | 로봇 `~/.bashrc`의 `# roport-domain` 표시 줄을 지우고 `export ROS_DOMAIN_ID=<N>  # roport-domain` 한 줄을 끝에 추가 |
| 콘솔 켜기(테스트 관제) | `ssh pinky@<IP>`로 `bash -ic 'exec ros2 launch pinky_test_console console.launch.py mode:=test\|ir robot:=<IR 기준값>'`를 `nohup`으로 실행(출력 `~/roport_logs/console_<mode>.log`). 등록의 RMW를 고르면 앞에 `RMW_IMPLEMENTATION=` 붙임. 이미 켜져 있으면 거부 |
| 콘솔 끄기 | 그 launch에만 SIGINT, 10 s 안에 안 끝나면 SIGTERM. bringup은 건드리지 않음 |
| 로그 파일 가져오기 | `scp pinky@<IP>:~/roport_logs/<파일> <fleet_log_dir>/<로봇>/`. ROS 로그 폴더는 `ssh … tar czf - -C ~/.ros/log <폴더>`를 `.tar.gz`로 저장 |

도메인 적용은 **로봇 `~/.bashrc`를 바꿉니다**(사용자 허용 2026-11-03).
- 표시(`# roport-domain`)가 없는 다른 줄(이전 사용자의 `ROS_DOMAIN_ID`·CycloneDDS 설정 등)은 지우지 않습니다. 끝에 넣은 줄이 나중에 읽혀 우선합니다.
- 새 터미널부터 적용됩니다.
- 되돌리려면 로봇에서 `sed -i '/# roport-domain$/d' ~/.bashrc`를 실행합니다.
- 기체별 값(2026-11-03 결정): d660 14 · 6422 15 · 764e 16.

보안: 이 서버는 기본으로 `127.0.0.1`에만 엽니다. `Host` 헤더가 `127.0.0.1:<port>`·`localhost:<port>`가 아니면 403입니다(DNS rebinding 방지).
SSH는 키 파일만 쓰고(비밀번호 입력 없음) 명령은 인자 목록으로 실행합니다. IP·이름·브랜치·도메인 값은 서버에서 검사합니다.
로봇 상태·경로 명령·ROS 로그는 각 로봇 콘솔(`mode:=test`·`route`, `http://<IP>:8080`)의 `/api/route`·`/api/logs`를 1초마다 부릅니다. 로봇 콘솔에는 로그인이 없으므로 강의실 Wi-Fi 안에서만 씁니다.

테스트 관제(`fleet.html`) 사용 순서:
1. 오른쪽 표에서 로봇을 고릅니다.
2. 「출발」에서 P1·P2·P5·P6(또는 마지막으로 도착한 「현재 위치」)를 고르고, 지도에서 주황 테두리 노드(정차 노드 N01·N11·N12·P1·P2·P5·P6)를 누릅니다. 경로는 선 중심 거리 최단 경로로 정합니다.
3. 로봇을 출발 노드에 첫 구간 방향(칸 바깥)으로 세운 뒤 **시작**.
- 「경유 상차」에서 P4(N01)·P7(N11)을 고르면 목적지 전에 그 노드에 들러 상차 완료까지 기다렸다 이어 감(예: 출발 P5, 경유 P4, 목적지 P7).
- 「직선 속도」·「회전 속도」 슬라이더 → **속도 적용**: 선택한 로봇의 line_nav 속도를 바로 바꿈(주행 중 가능). 범위는 로봇 launch의 `max_linear`·`max_angular`(기본 6 cm/s·0.6 rad/s)까지. 경로 주행 화면(`route.html`)에도 같은 조절이 있음.
- 지도의 로봇 위치는 `line_nav/state`의 구간·이동 거리로 추정한 값입니다(실제 위치 측정 아님).
- **복귀**: 로봇이 노드에서 끝났으면(`arrived`·`handoff`) 그 노드에서 Pinky 설정의 「복귀 칸」(P1·P2·P5·P6)까지 최단 경로로 시작합니다. N12 handoff 뒤처럼 선 끝을 보고 섰으면 line_nav가 먼저 반 바퀴 돕니다.
  - 주행 중·상차 대기 중이면 거부합니다(끝나거나 정지한 뒤 다시). 정지·오류 뒤에는 위치를 모르므로 거부합니다(로봇을 노드에 두고 「출발」을 골라 시작).
  - 「전체 복귀」는 복귀 칸이 있는 모든 로봇에 같은 규칙으로 보냅니다. **로봇끼리 충돌 회피는 없으므로** 경로가 겹치지 않는지 보고 누릅니다.
- 로그 = 로봇 ROS 로그(`/rosout`, 구분 `pinky`)·line_nav 이벤트·state 변화·콘솔 연결 변화·보낸 명령·설치 작업. 화면은 최근 1500줄, 「CSV 저장」은 서버 메모리의 최근 3000줄. 모든 줄은 `fleet_log_dir`(기본 `~/roport_fleet_logs/fleet_YYYYMMDD.csv`)에 자동으로 이어 씁니다. `=`·`+`·`-`·`@`로 시작하는 값은 표 계산 프로그램이 수식으로 읽지 않게 앞에 `'`를 붙입니다.

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

## 정밀 도킹 (mode:=fleet 「정밀 도킹」 탭, FFT1-112)

**실로봇·실카메라 미검증**(가짜 카메라·가짜 로봇 상태로 화면·API만 확인). 로봇 쪽 `dock` 명령·교차선 출발은 `roport_line_nav`가 처리합니다.

통합 화면 위쪽 로봇을 고르고 「정밀 도킹」 탭(`dock.html`, 노트북 화면)을 엽니다.
1. 탑뷰 카메라를 고르고 「실시간」 또는 「사진 1장」. 보정(`dock_dir`의 `calib_*.json`, 없으면 「보정 JSON 직접 불러오기」)으로 지도·로봇 박스를 겹칩니다.
   주황 = 지금 코드 예상, 초록 = 목표(끌기·앞쪽 원 끌기·방향키·`[` `]`), 파랑 점선 = 로봇에 적용된 값(`line_nav/state`의 `dock`), 보라 = 주행 중 `pose_est`, 빨강 = 실제(사진 위에 끌어 맞춤).
2. ①(가로선에서 멈춘 뒤 선 따라)·②(반시계 회전)·③(새 방향 직진) 계산값이 범위(|①|·|③| ≤ 6 cm, 60° ≤ ② ≤ 120°) 안이면 「로봇에 적용」 → 로봇에 `{"cmd": "dock", "node", "advance_cm", "rotate_deg", "move_cm"}`. 「로봇 값 지우기」 = `reset`. 상차 뒤에는 ③·①을 되돌린 다음 선을 찾습니다(③ 되돌림 → 시계 방향 ② 회전 → ① 되돌림 → 선 찾기, line_nav).
3. 시험 주행: N07에서 남쪽을 보고 IR 3개가 동서 선 위 → N11(N07 N23 N19 N11) 또는 N01(N07 N18 N01). `start_cross: true, start_facing: "S"`로 시작. 상차 완료·정지·P5 복귀.
4. 「도착 사진 찍기」(지금 장 고정) → 「실제 위치 표시」로 빨간 박스를 로봇에 맞춤 → 실제 − 목표(선 따라·옆·각) → 「결과 기록」(`~/roport_fleet_logs/dock/dock_<노드>_<시각>.json`, 사진은 저장 안 함).

카메라 설정 `~/roport_dock/cameras.json`(없으면 아래 두 대가 기본, 보정 파일은 이 폴더에 복사):

```json
[{"id": "omx12", "label": "OMX1·2 탑뷰", "host": "omen", "device": "/dev/video4", "size": "1280x720", "node": "N11", "calib": "calib_N11_20261107_184517.json"},
 {"id": "omx3", "label": "OMX3 탑뷰", "host": "gpu", "device": "/dev/video0", "size": "1280x720", "node": "N01", "calib": "calib_N01_20261107_190226.json"}]
```

- `host`는 노트북 `~/.ssh/config` 별칭(키 로그인). `StrictHostKeyChecking=yes`라서 처음 한 번은 터미널에서 `ssh <별칭>`으로 호스트 키를 확인·등록합니다.
- 실행: `ssh -o BatchMode=yes -o StrictHostKeyChecking=yes -o ConnectTimeout=6 … -- <host> ffmpeg -hide_banner -loglevel error -f v4l2 -input_format mjpeg -video_size <size> -i <device>`
  + 사진 `-frames:v 1 -c:v mjpeg -q:v 3 -f image2pipe -`(15 s 제한) / 실시간 `-an -vf fps=5 -c:v mjpeg -q:v 5 -f mpjpeg -`. host·device·size는 정규식으로 검사하고, 셸 문자열 없이 인자 목록으로 실행합니다.
- 카메라당 ssh ffmpeg 하나를 여러 화면이 같이 봅니다. 실시간 중 사진 요청은 그 장을 돌려줍니다. 보는 화면이 없으면 3 s 뒤, 서버를 끄면 바로 이 서버가 띄운 ssh만 끕니다.
- 카메라 사진·실시간 GET은 ssh·장치를 여는 부작용이 있어 브라우저 `Sec-Fetch-Site`가 `same-origin`·`none`이 아니면 403입니다(다른 사이트의 `<img>` 요청 차단). 이 헤더가 없는 curl 등은 허용합니다.
- 카메라가 OMX 팀 프로그램과 같이 쓰입니다. 장치가 사용 중이면 409 「사용 중」을 보여 주고 **다른 프로그램은 끄지 않습니다**. 반대로 이 탭이 실시간으로 보는 동안에는 OMX 프로그램이 카메라를 못 열 수 있으니 다 보면 끕니다(10분 뒤 자동으로 끔).

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
| POST | `/api/marker/record` `{distance_cm, duration, marker_id?, note?}` | 거리별 기록(1–30 s) |
| GET | `/api/marker/records`, `/api/marker/records.csv` | 거리별 기록 목록 / CSV 다운로드 |
| GET | `/api/amcl?map_v=..` | 지도(버전이 바뀐 경우만 데이터)·스캔 점·정합 점수·AMCL 위치·노드·최근 기록 |
| POST | `/api/amcl/initialpose` `{node}` 또는 `{x_cm, y_cm, yaw_deg}` | 노드 또는 실측 좌표로 AMCL 초기 위치 |
| POST | `/api/amcl/nomotion` | AMCL 제자리 갱신 한 번 |
| POST | `/api/amcl/record` `{label, truth?: {x_cm, y_cm, yaw_deg?}}` / GET `/api/amcl/records.csv` | 기록 한 줄 / CSV 다운로드 |
| GET | `/api/ir` | IR 최근값·주기·최근 10 s 기록·기록 진행 상태 |
| POST | `/api/ir/record` `{label, duration}` / `/api/ir/cancel` | IR 기록 시작(1–60 s) / 취소 |
| GET | `/api/ir/records`, `/api/ir/records/<파일>` | 기록 목록·통계·참고 임계값 / CSV 다운로드 |
| GET | `/api/route`, `/api/route/graph` | `line_nav/state`(없으면 `stale`) / 노드 그래프·경로 예시 |
| POST | `/api/route/command` `{cmd: start\|stop\|load_done\|speed\|dock, route?, start_offset_cm?, start_cross?, start_facing?, base_speed?, rotate_speed?}` | `line_nav/command`로 보냄 (`mode:=route`·`test`). `dock` = `{node: N01\|N11\|N21\|N22, advance_cm, rotate_deg, move_cm}` 또는 `{node, reset: true}`. `start_facing`(E·N·W·S)은 `start_cross: true`와만 |
| GET | `/api/fleet`, `/api/fleet/jobs`, `/api/fleet/branches` | (`mode:=fleet`) 등록 목록·로봇 상태·작업 기록 / 작업 / `origin/*` 브랜치 |
| POST | `/api/fleet/robots` `{name, ip, role, domain, calib, rmw?, home?}`, `/api/fleet/robots/delete` `{name}` | 등록·수정 / 삭제 |
| POST | `/api/fleet/job` `{name, action: check\|read_domain\|set_domain\|deploy\|build, ref?, release?, domain?}` | SSH·git 작업(로봇마다 한 번에 하나) |
| POST | `/api/fleet/route` `{name, cmd, route?}`, `/api/fleet/stop_all` | 로봇 콘솔로 경로 명령 / 전체 정지 |
| POST | `/api/fleet/return` `{name?}` | 복귀(이름 없으면 전체). 로봇별 `{ok, msg, route}` |
| GET | `/api/route/plan?from=<노드>&to=<정차 노드>&via=<정차 노드>?` | 최단 경로(노드 id 목록). `via`가 있으면 그 노드를 경유(상차 대기) |
| GET | `/api/fleet/logs?after=<seq>`, `/api/fleet/logs.csv?robot=<이름>` | 로그(seq 이후) / CSV 다운로드 |

| GET | `/api/logs?after=<seq>`, `/api/logs/files` | (로봇) `/rosout` 최근 2000줄 / 경로 상세 기록·ROS 로그 폴더 목록 |
| GET | `/api/fleet/files?name=<로봇>` | (`mode:=fleet`) 그 로봇의 로그 파일 목록 |
| POST | `/api/fleet/job` `{name, action: fetch_log\|fetch_roslog\|console_start\|console_stop, file?, dir?, mode?}` | 로그 파일 가져오기 / 콘솔 켜기·끄기 |
| GET | `/api/dock/cameras`, `/api/dock/calib/<id>` | (`mode:=fleet`) 탑뷰 카메라 목록·중계 상태 / 보정(`floor_to_img` 3×3) |
| GET | `/api/dock/camera/<id>/snapshot.jpg`, `/api/dock/camera/<id>/stream` | 사진 1장 / MJPEG(`multipart/x-mixed-replace`). 장치 사용 중 409, SSH 실패 502, 시간 초과 504 |
| POST·GET | `/api/dock/result`, `/api/dock/results` | 정밀 도킹 결과 JSON 저장(`fleet_log_dir/dock`, 홈 밖 폴더는 거부) / 최근 50건 |
| GET·POST | `/robot/<이름>/<화면 또는 api/...>` | (`mode:=fleet`) 등록된 로봇 콘솔의 화면 파일·`api/`만 중계(통합 화면 탭) |

POST는 `Content-Type: application/json`만 받습니다. 다른 웹사이트가 숨은 form으로 로봇을 움직이는 CSRF를 막기 위해서입니다.

## 알려진 제한

- 통합 화면은 로봇 화면을 노트북과 같은 출처로 중계합니다. 그래서 로봇 콘솔이 보낸 화면은 노트북 서버(SSH 작업 권한)와 같은 권한으로 실행됩니다. 팀 로봇의 콘솔만 등록하세요.

- **인증·HTTPS 없음**(위 경고). Flask 개발 서버를 그대로 씁니다(원본과 동일).
- `slam_toolbox`의 SaveMap은 받은 경로를 쉘 명령(`map_saver_cli -f …`)으로 실행합니다. 그래서 지도 이름은 `A-Z a-z 0-9 _ -`만, 저장 폴더 경로도 안전한 문자만 허용합니다.
- 조종 입력으로 Nav2 목표를 취소한 직후에는 Nav2 velocity_smoother가 감속 명령을 잠시 더 `/cmd_vel`에 보냅니다.
  Gazebo에서 잰 값은 취소 후 0.32 s 동안 −1.0 → 0 rad/s로 줄어드는 명령 7개였습니다. 그 뒤로는 조종 명령만 나갑니다.
- 종료 로그 `Published zero velocity on shutdown (subscribers=N, acked=…)`에서 N은 `/cmd_vel` 구독자 수(로봇이면 bringup 포함)입니다.
  `acked=True`면 모든 구독자가 0을 받았다는 확인입니다. 죽은 구독자가 디스커버리에 남아 있으면 `False`가 나올 수 있습니다.
- 콘솔 프로세스 RSS는 x86 노트북 측정값으로 약 100 MB입니다(Python + rclpy + Flask). Pi에서는 직접 확인하세요.
- local costmap은 odom 프레임이지만 원본과 같이 map 좌표로 그립니다. SLAM 드리프트가 크면 조금 어긋나 보일 수 있습니다.
- `mode:=slam`은 요구사항대로 slam_toolbox만 띄웁니다(원본 `web_slam`은 Nav2도 함께 띄움). 그래서 이 모드에서는 Nav2 목표를 쓸 수 없습니다.

## 스마트카트 상태 화면

`smartcart.html`은 기존 `mode:=amcl` 콘솔에서 scan/odom/TF·지도·AMCL 위치와 정지 관측을 보여준다.
이미 map_server/AMCL이 실행 중이면 `web_server`만 같은 ROS domain에서 실행한다(중복 bringup/launch 금지).
H1/H2 초기 위치는 사용자가 실제 방향을 직접 입력하고 배치 확인을 눌렀을 때만 전달한다.
서버에서도 최신 scan/odom과 연속 정지·Nav2 비주행 조건을 검사하며 초기 위치 이외의 주행 목표는 보내지 않는다.
실기 게이트 좌표 미측정·배송 실행부 미연결로 배송 버튼은 비활성화되어 있다. 실시간 지도는 기존 AMCL 화면을 연다.

### 관제 trial_guard와 함께 보기

`trial_guard`만 `/cmd_vel` 발행자가 되도록, 기존 map_server/AMCL이 실행 중인 PC에서 콘솔 단독 실행 시 다음 옵션을 사용한다.
같은 workspace/ROS domain을 source한 뒤 필요한 `layout_yaml`·loopback 포트를 지정한다. 기본값은 기존 조종 모드(`true`)다.

```bash
ros2 run pinky_test_console web_server --ros-args \
  -p mode:=amcl -p ip:=127.0.0.1 -p port:=8080 \
  -p layout_yaml:=/absolute/path/roport_airport.yaml -p teleop_enabled:=false
```

`false`에서는 `cmd_vel` 발행자 자체를 만들지 않고 종료 시에도 0 속도를 보내지 않는다.
수동 조종·직접 주행 목표·경로 명령·웹 정지/취소 요청은 HTTP 403으로 거부한다. 지도·센서·AMCL 초기 위치 설정은 유지한다.
이 화면은 현장 비상정지가 아니다. 실제 정지는 운영자가 확인한 별도 현장 정지 수단으로 수행한다.
