# roport_gz_world

RO:PORT 시연 공간 실측 배치(v13)를 Pinky Pro용 Gazebo world로 만든 패키지입니다.
같은 배치 데이터에서 Nav2 지도(pgm)와 실측 도면(노드 그래프 포함)도 만듭니다.
world 좌표는 SLAM 지도(`map` 프레임) 좌표계에 맞춰 두었습니다.

| 파일 | 내용 |
|---|---|
| `layout/roport_airport.yaml` | **원본 데이터.** 벽·물체·라인 노드 그래프·마커를 실측 좌표(cm)로 기록. 치수마다 출처 주석 |
| `worlds/roport_airport.sdf` | 위 데이터로 **자동 생성**한 world. 직접 고치지 않음 |
| `../../maps/airport_map.{pgm,yaml,png}` | 위 데이터로 **자동 생성**한 Nav2 지도(공용 `maps/`). 설치 시 이 패키지 `share/.../maps`에 함께 들어감 |
| `../../maps/map_measured_v13.png` | 위 데이터로 그린 실측 도면 |
| `launch/roport_gz_world.launch.xml` | 원본 `pinky_gz_sim/launch_sim.launch.xml`을 이 world로 include |
| `roport_gz_world/` | 배치 데이터 읽기·좌표 변환(`layout.py`), world·지도·도면 생성 |

## 실행

```bash
cd services/roport-pinky
source /opt/ros/jazzy/setup.bash
# Nav2 launch(pinky_navigation)와 웹 콘솔(pinky_test_console)은 roport_gz_world 의존이 아니므로 함께 지정
colcon build --packages-up-to roport_gz_world pinky_navigation pinky_test_console
source install/setup.bash

# 터미널 1: Gazebo (GUI 포함, 원본 launch_sim 그대로)
ros2 launch roport_gz_world roport_gz_world.launch.xml

# 터미널 2: 실측 지도로 Nav2 (원본 pinky_navigation 시뮬레이션용 launch)
ros2 launch pinky_navigation gz_bringup_launch.xml \
  map:=$(ros2 pkg prefix --share roport_gz_world)/maps/airport_map.yaml
```

- 로봇은 world (0, 0), +x 방향에 나타납니다. 이 자리는 SLAM 지도 원점, 즉 실제 로봇이 SLAM을 시작한 위치(실측 약 x 152, y 202 cm)입니다. AMCL 초기 위치를 (0, 0, 0)으로 주면 됩니다.
- 웹 콘솔(`pinky_test_console`, 위 빌드 명령에 포함)로 조종·Nav2를 하려면 터미널 2 대신 다음을 실행합니다.
  `ros2 launch pinky_test_console console.launch.py mode:=nav map:=<위 지도 경로> use_sim_time:=true`
- 필요한 apt 패키지: `ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-xacro`

## 배치 수정

1. `layout/roport_airport.yaml`의 값을 고칩니다(단위 cm, 원점 = 콘센트 벽·통로 쪽 모서리, x = 길이, y = 창가).
2. 패키지 폴더에서 다시 생성합니다.
   ```bash
   cd services/roport-pinky/src/roport_gz_world
   python3 -m roport_gz_world.generate_world    # world
   python3 -m roport_gz_world.generate_map      # Nav2 지도
   python3 -m roport_gz_world.draw_layout       # 도면 (matplotlib·fonts-noto-cjk 필요)
   ```
3. `python3 -m pytest test`로 확인합니다. world나 지도를 다시 생성하지 않으면 `test_committed_world_matches_layout`·`test_committed_map_matches_layout`이 실패합니다.

## 라인 노드 그래프

`nodes`에 노드 id·좌표(테이프 중심선)·연결·운영 위치(P1~P7)를 적습니다. 바닥 테이프는 연결 구간과 정차 노드의 가로선(12 cm)에서 만들어집니다.

- 번호는 Map 페이지 노드 제안 v0.1을 유지합니다. N13·N20·H3는 삭제했습니다.
- N12(간선 끝)가 라인 주행 → Nav2 전환 노드입니다. 대기·충전 위치가 아닙니다.
- 스마트카트 대기·충전은 P1(H1)·P2(H2), 물류 Pinky 대기·충전은 P5(N16)·P6(N17)입니다. P4(N01)는 창고 상차, P7(N11)은 인도장(공용)입니다.

## 모델링 규칙

- 가벽 두께 0.5 cm·높이 24 cm, 테이프 폭 24 mm(두께 1 mm, 흰색)입니다. 물체 높이는 벽 높이와 같게 둡니다. 실측 높이가 있는 물체(창고박스 11 cm)는 그 값을 씁니다.
- 벽·물체는 충돌이 있고, 라인·마커는 충돌이 없습니다.
- **Nav2 지도에는 벽과 라이다(12.5 cm)보다 높은 물체만 점유로 그립니다.** 테이프·마커·창고박스는 라이다에 안 보이므로 그리지 않습니다. 벽 안쪽은 빈 곳, 벽 바깥은 미지입니다.
- **Gazebo GPU 라이다는 충돌 모델이 아니라 visual을 감지합니다.** 충돌이 없는 마커 표시판도 라이다 평면(바닥에서 12.5 cm)에 걸리면 스캔에 잡힙니다. 그래서 마커 표시판 윗면을 10 cm에 둡니다.
- 실측 → 지도 좌표 변환(`frame`)은 2026-10-27 SLAM 지도의 벽 점유 칸에 실측 벽을 맞춰 정한 값(x −1.520 m, y −2.0225 m, yaw 0.10°)입니다. v13 실측 벽과 SLAM 지도 벽의 거리는 벽마다 평균 1.4~2.8 cm, 최대 6.2 cm입니다(지도 칸 5 cm).

`provisional: true`는 이번 실측에 없는 값입니다(위치 추정·가정). 도면에서 주황색으로 표시합니다.

| 항목 | 가정 |
|---|---|
| P4 분기(N18·N01) x | 루프 가운데 67.1 |
| P7 분기(N19·N11) x, 02 선반·OMX1·2 받침 x | v12 추정 231 기준 가운데 정렬 |
| OMX1·2 받침 모양 | 좌우 대칭 U자 |
| OMX3 받침 | 오른쪽 끝 = 왼쪽 벽에서 127 |
| 03 카메라 삼각대 x | v12 위치(03 왼쪽 벽에서 64) |
| 마커 위치 | 전부 추정 |

## 검증

| 항목 | 결과 |
|---|---|
| 단위 테스트 (`python3 -m pytest test`) | 실측 치수(a·b·c·D·O·Q·R·S·U·W·ㅈ·ㄱ·I·입구), 노드 그래프(연결 대칭·한 그래프·정차 노드 = 가로선), world·지도와 배치 일치, SLAM 지도와의 벽 정합, 지도 픽셀, `gz sdf -k` |
| v12 시뮬레이션 스캔 ↔ SLAM 지도 (이전 배치) | 6개 위치에서 라이다 끝점과 지도 점유 칸의 거리 중앙값 2.2 cm, 5 cm 이내 95.7% |

실측 지도는 실로봇 AMCL로 아직 확인하지 않았습니다. 시뮬레이션 결과는 실로봇 주행 검증을 대신하지 않습니다.
