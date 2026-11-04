# roport_line_nav

IR 라인 노드 경로 주행(Jira FFT1-117). 물류 Pinky와 스마트카트가 같이 쓴다.
노드 그래프는 [roport_gz_world/layout/roport_airport.yaml](../roport_gz_world/layout/roport_airport.yaml)을 읽고,
IR 판정·기체별 임계값은 [roport_pinky_line](../roport_pinky_line/)(라인 추종 v0)을 쓴다.

실로봇에서는 아직 확인하지 않았다. 단위 테스트는 2D 시뮬레이터(`test/sim.py`, 실측 테이프 배치 + IR 모델)로 한다.
Gazebo에는 IR 센서가 없다. 시뮬레이터 결과는 실로봇 주행 검증을 대신하지 않는다.

## 동작

| 단계 | 내용 |
| --- | --- |
| 경로 | 노드 id 목록(예: `N16 N14 N02 N10 N18 N01`). 이웃 노드끼리만, 중간 U턴 없음, 마지막은 가로선이 있는 정차 노드 |
| 직선 | 라인 추종 v0와 같은 PD 조향, 기본 0.03 m/s |
| 노드 감지 | 정차 노드 = IR 3개 모두 테이프. 그 밖의 노드 = 가운데 + 옆 선 쪽 채널이 테이프. **odom 이동 거리가 예상 거리 ±30%(+3 cm) 안일 때만** 노드로 인정(2026-11-03 결정, 테이프 추가 없음) |
| 직진 노드 | 방향을 유지하며 5 cm 통과(옆 선 쪽으로 끌려가지 않게) |
| 90° 회전 | 옆 선 감지 → 4.2 cm(IR ↔ 바퀴 축 3 + 테이프 반폭 1.2) 더 직진 → 제자리 회전. 60° 이상 돌고 가운데 채널이 선 위·좌우 오차 0.25 이하이면 다음 구간. 120°까지 못 찾으면 정지 |
| N01(P4)·N11(P7) 도착 | 가로선에서 정지 → 반시계 90° → **상차 완료(`load_done`)까지 대기** → 반시계 회전해 진입선을 찾음(합계 약 180°) |
| P1·P2·P5·P6 도착 | 가로선에서 정지 → 반시계 회전으로 진입선 확인(172° 이상, 210°까지) → 0.02 m/s 후진 → IR 3개가 가로선을 보면 정지. 12 cm 넘게 후진하면 정지 |
| N12 도착 | 정지하고 `handoff`(Nav2 전환 가능) 보고. Nav2 주행은 범위 밖 |

오류(`error`)로 멈추는 경우: 예상 거리를 지나도 노드 없음(`node_missed`), 라인 놓침 1초 이상, 회전 중 선을 못 찾음,
후진 초과, IR 끊김 0.3초(`no_ir`), odom 끊김 0.5초(`no_odom`). 정지·오류 때 0 속도를 한 번 발행하고 그 뒤로는 발행하지 않는다.

경로가 끝나면 로봇은 다음 출발 방향을 보고 선다. 상차 노드는 진입선 쪽, 대기·충전 칸은 칸 바깥쪽이다.
그래서 다음 경로를 그 노드에서 바로 시작할 수 있다.

이어서 출발: 실행부는 마지막으로 끝난 노드·방향·센서 위치를 기억한다.
- 끝난 노드에서 다시 출발하면 그 센서 위치로 첫 구간 거리 창을 정한다(상차 노드 7.2 cm, 대기 칸 1.2 cm).
- 첫 구간이 로봇이 보는 방향과 반대이면(예: N12 handoff 뒤 `N12 → N08 …`) 먼저 반시계로 반 바퀴(172~210°) 돌아 진입선을 찾는다(이벤트 `pre_rotate`).
- 정지·오류 뒤에는 기억을 지운다. 사람이 로봇을 옮긴 경우 첫 구간 방향으로 세우고 정지한 뒤 시작한다.
- 기억이 없으면 가로선 노드 출발은 센서가 가로선 위(0 cm)에 있다고 보고 거리 창을 쓴다. 그 밖의 노드는 첫 구간 최소 거리 제한이 없다.

## 토픽

| 토픽 | 형식 | 내용 |
| --- | --- | --- |
| `ir_sensor/range` | `std_msgs/UInt16MultiArray` | 입력. `pinky_sensor_adc` 약 20 Hz |
| `odom` | `nav_msgs/Odometry` | 입력. `pinky_bringup` 엔코더 |
| `line_nav/command` | `std_msgs/String` JSON | `{"cmd":"start","route":["N16","N14",...]}` · `{"cmd":"stop"}` · `{"cmd":"load_done"}` |
| `cmd_vel` | `geometry_msgs/Twist` | 출력. 주행 중일 때만, `dry_run` 이면 없음 |
| `line_nav/state` | `std_msgs/String` JSON | `state`·`reason`·`route`·`leg`·`traveled_cm`·`window_cm`·`events`·`ir`·`last_command`·`graph_version` |

`state`: `idle` · `follow` · `pass` · `advance` · `rotate` · `loading` · `reverse` · `arrived` · `handoff` · `stopped` · `error`.
`start_offset_cm`(선택, 0~50)은 출발 노드에서 센서까지 거리다. 없으면 첫 구간은 최소 거리 제한 없이 감지한다.

## 실행 (실로봇, 사용자 수행)

`line_follower`와 같이 띄우지 않는다(둘 다 `cmd_vel` 발행). pinky_6422는 명령 앞에 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`를 붙인다.

```bash
# 빌드: roport_gz_world(노드 그래프), roport_pinky_line(IR 판정), roport_line_nav
colcon build --packages-select roport_gz_world roport_pinky_line roport_line_nav && source install/setup.bash
# 터미널 1 — 모터·odom
ros2 launch pinky_bringup bringup_robot.launch.xml
# 터미널 2 — IR 노드 + 경로 주행
ros2 launch roport_line_nav line_nav.launch.py robot:=pinky_d660
# 터미널 3 — 상태
ros2 topic echo --full-length /line_nav/state
# 터미널 4 — 명령
ros2 topic pub --once /line_nav/command std_msgs/msg/String "{data: '{\"cmd\": \"start\", \"route\": [\"N14\", \"N16\"]}'}"
ros2 topic pub --once /line_nav/command std_msgs/msg/String "{data: '{\"cmd\": \"load_done\"}'}"
ros2 topic pub --once /line_nav/command std_msgs/msg/String "{data: '{\"cmd\": \"stop\"}'}"
```

테스트 웹 콘솔(`pinky_test_console`)의 `mode:=test`(경로+IR)·`mode:=route` 화면에서도 경로 지정·시작·정지·상차 완료를 할 수 있다.

경로 상세 기록: 경로 하나마다 `~/roport_logs/route_<YYYYmmdd_HHMMSS>_<출발>-<도착>.csv`(IR 20 Hz 한 줄씩: state·구간·이동 거리·회전·IR 원시값·on_line·오차·cmd_vel·odom·이벤트)를 쓰고 최근 50개만 남긴다. 노드 파라미터 `log_dir`(빈 값이면 기록 안 함)·`log_keep`. 쓰기 오류가 나면 기록만 끄고 주행은 계속한다.

조정 인자(launch): `base_speed`, `max_linear`, `max_angular`, `rotate_speed`, `reverse_speed`, `ir_ahead_cm`(실측 3), `turn_advance_cm`(4.2), `layout_file`.

## 빌드·테스트

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select roport_pinky_line roport_line_nav
source install/setup.bash
python3 -m pytest src/roport_line_nav/test -q
```

`test_executor.py`는 시뮬레이터로 Map 페이지 경로 예시 5개를 끝까지 달린다. 엔코더 ±7%와 테이프 위치 차이 최대 3.6 cm를 넣은 경우도 확인한다.
노드 놓침·정지 명령·odom 없음도 확인한다. `test_node_ros.py`는 `ROS_DOMAIN_ID` 197과 전용 namespace를 써서 실제 로봇 `/cmd_vel`에 닿지 않는다.

## 한계

- 옆 선 감지는 「가운데 + 옆 채널」 패턴이라, 직선에서 라인이 가운데와 옆 센서 사이에 오면 같은 패턴이 나온다. 이동 거리 창(±30%) 밖에서는 무시하고, 창 안에서 잘못 감지하면 회전 중 선을 못 찾아 정지한다.
- 90° 회전·후진 거리는 odom(엔코더)만으로 잰다. 바퀴 미끄러짐은 보정하지 않는다.
- 실측과 테이프 위치가 다르면 예상 거리 창이 틀어진다. 배치 yaml의 노드 좌표를 실측으로 맞춰 둔다.
