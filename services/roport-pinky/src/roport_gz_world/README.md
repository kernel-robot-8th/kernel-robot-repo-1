# roport_gz_world

RO:PORT 시연 공간(실측 맵 v11)을 Pinky Pro용 Gazebo world로 만든 패키지입니다.
world 좌표를 실제 SLAM 지도(`maps/airport_map.yaml`) 좌표계에 맞춰 두었습니다.
그래서 시뮬레이션에서도 실제 지도를 그대로 Nav2에 쓸 수 있습니다.

| 파일 | 내용 |
|---|---|
| `layout/roport_airport.yaml` | **원본 데이터.** 벽·물체·바닥 라인·마커를 실측 좌표(cm)로 기록. 치수마다 출처 주석 |
| `layout/map_measured_v11.png` | 실측 맵 v11 (원본 도면) |
| `worlds/roport_airport.sdf` | 위 데이터로 **자동 생성**한 world. 직접 고치지 않음 |
| `maps/airport_map.yaml`, `.pgm` | 실제 Pinky로 만든 SLAM 지도 (5 cm/칸) |
| `launch/roport_gz_world.launch.xml` | 원본 `pinky_gz_sim/launch_sim.launch.xml`을 이 world로 include |
| `roport_gz_world/` | 배치 데이터 읽기·좌표 변환·SDF 생성 |

## 실행

```bash
cd services/roport-pinky
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to roport_gz_world
source install/setup.bash

# 터미널 1: Gazebo (GUI 포함, 원본 launch_sim 그대로)
ros2 launch roport_gz_world roport_gz_world.launch.xml

# 터미널 2: 실제 SLAM 지도로 Nav2 (원본 pinky_navigation 시뮬레이션용 launch)
ros2 launch pinky_navigation gz_bringup_launch.xml \
  map:=$(ros2 pkg prefix --share roport_gz_world)/maps/airport_map.yaml
```

- 로봇은 world (0, 0), +x 방향에 나타납니다. 이 자리는 SLAM 지도 원점, 즉 실제 로봇이 SLAM을 시작한 위치(실측 약 x 152, y 202 cm)입니다. AMCL 초기 위치를 (0, 0, 0)으로 주면 됩니다.
- 웹 콘솔(`pinky_test_console`)로 조종·Nav2를 하려면 터미널 2 대신 다음을 실행합니다.
  `ros2 launch pinky_test_console console.launch.py mode:=nav map:=<위 지도 경로> use_sim_time:=true`
- 필요한 apt 패키지: `ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-xacro`

## 배치 수정 (OMX 구역 확정 등)

1. `layout/roport_airport.yaml`의 값을 고칩니다(단위 cm, 원점 = 콘센트 벽·통로 쪽 모서리, x = 길이, y = 창가).
2. 패키지 폴더에서 world를 다시 생성합니다.
   ```bash
   cd services/roport-pinky/src/roport_gz_world
   python3 -m roport_gz_world.generate_world
   ```
3. `python3 -m pytest test`로 확인합니다. world를 다시 생성하지 않으면 `test_committed_world_matches_layout`이 실패합니다.

`provisional: true`가 붙은 값은 실측 전 임시값입니다(Jira FFT1-96, FFT1-97).

| 항목 | 임시값 | 근거 |
|---|---|---|
| 가벽 높이 | 20 cm | 사진 추정 |
| 물체 높이 | 15 cm | Pinky 라이다 높이(12.5 cm)보다 높아야 SLAM 지도에 보인 것과 일치 |
| 아래쪽 칸막이 2개 길이 | 54 / 49 cm | 도면 픽셀 추정 |
| 선반 테이블 | x 192~257, 깊이 7.5 cm | 도면에 치수가 없어 SLAM 지도 관측값 사용 |
| OMX3 받침 | 46.8 × 45.5, 높이 8 cm, 위치 추정 | 도면 점선. SLAM 지도에 보이지 않아 라이다보다 낮게 둠 |
| P4·P5·P6·인도장 연결 라인, 마커 | 도면 위치 | 위치 추정 또는 크기 미정 |

## 모델링 규칙

- 가벽 두께 0.5 cm, 바닥 라인 폭 2 cm(두께 1 mm, 흰색)입니다. 벽·물체는 충돌이 있고, 라인·마커는 충돌이 없습니다.
- **Gazebo GPU 라이다는 충돌 모델이 아니라 visual을 감지합니다.** 충돌이 없는 마커 표시판도 라이다 평면(바닥에서 12.5 cm)에 걸리면 스캔에 잡힙니다. 그래서 마커 표시판은 윗면을 10 cm에 둡니다. 실제 SLAM 지도에 마커가 보이지 않은 것과 맞추기 위해서입니다.
- 실측 → 지도 좌표 변환(`frame`)은 SLAM 지도의 벽 점유 칸에 실측 벽 선분을 맞춰 정했습니다. 결과는 x −1.520 m, y −2.0225 m, yaw 0.10°이고, 벽 평균 오차 1.7 cm, 최대 5.4 cm입니다.

## 검증

| 항목 | 결과 |
|---|---|
| 단위 테스트 23개 (`python3 -m pytest test`) | 통과. 외곽 443×220, 벽 두께·라인 폭, 스폰 위치 여유, world·배치 일치, 실제 지도와의 벽 정합, `gz sdf -k` |
| 시뮬레이션 스캔 ↔ 실제 SLAM 지도 | 6개 위치에서 라이다 끝점과 실제 지도 점유 칸의 거리: 중앙값 2.2 cm, 5 cm 이내 95.7%, 10 cm 이내 99.9% |
| 실제 지도로 Nav2 주행 (시뮬레이션) | SLAM 시작점 → 입구 → P2 쪽 목표 `Goal succeeded`(45 s). 도착 오차 약 21 cm로 원본 설정 허용 0.25 m 이내. 중간에 경로 계획 실패 1회, `Failed to make progress` 1회 후 회복 |

시뮬레이션 결과는 실로봇 주행 검증을 대신하지 않습니다. 강의실 실측 값과 OMX 구역은 확정 후 다시 반영해야 합니다.
