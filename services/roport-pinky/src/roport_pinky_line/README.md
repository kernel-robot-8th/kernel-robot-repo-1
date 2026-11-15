# roport_pinky_line

물류 Pinky IR 라인 추종 노드 v0 (Jira FFT1-115). IR 3채널로 라인 위치를 계산해 `cmd_vel`을 낸다.
노드 판정·90° 회전·경로 주행은 아직 없다(FFT1-117). 노드에서는 **정지만** 한다.

## 동작

| 항목 | 내용 |
| --- | --- |
| 입력 | `ir_sensor/range` (`std_msgs/UInt16MultiArray`, `pinky_sensor_adc` 약 20 Hz). data[0..2] = 왼쪽·가운데·오른쪽 |
| 켜기·끄기 | `line_follower/enable` (`std_msgs/Bool`). 처음에는 꺼진 상태 |
| 출력 | `cmd_vel` (`geometry_msgs/Twist`, 켜져 있을 때만), `line_follower/state` (`std_msgs/String`, JSON) |
| 판정 | 3샘플 중앙값 → 값 < 임계값이면 라인(히스테리시스 ±100) |
| 오차 | n = (카펫 − 값) ÷ (카펫 − 테이프), 0~1. 오차 = (n오른쪽 − n왼쪽) ÷ (n합). 양수 = 라인이 오른쪽 |
| 조향 | 각속도 = −(kp·오차 + kd·오차 변화율). 선속도 = base × (1 − 0.5·\|오차\|), 최소 0.02 m/s |
| 상한 | 기본 0.10 m/s·0.6 rad/s. 코드 고정 상한 0.15 m/s·1.5 rad/s(넘는 설정은 시작 시 오류) |

기체별 카펫·테이프·임계값은 [config/robots.yaml](config/robots.yaml)에 있다(2026-10-31 실측, 24 mm 2겹 기준 제안값).

### 상태(`state`)와 정지

| 상태 | 뜻 | 속도 |
| --- | --- | --- |
| `idle` | 꺼짐 | 발행 안 함 |
| `follow` | 라인 추종 | 계산값 |
| `line_lost` | 어느 채널도 라인이 아님 | 0. 라인이 다시 보이면 이어 감 |
| `node` | 세 채널 모두 라인 2샘플 연속(가로 테이프) | 0. 다시 켜야 출발(밟고 있는 노드는 지나감) |
| `no_ir` | 유효한 IR이 0.3초 넘게 안 옴. 채널 수가 3이 아니거나 값이 1~4095 밖인 메시지는 버림(원본 드라이버는 I²C 읽기 실패를 0으로 발행) | 0 |

끌 때, Ctrl+C·SIGTERM·SSH 끊김(SIGHUP) 종료 때도 0을 발행한다. `kill -9`·전원 차단·노드 강제 종료에는 0을 보낼 수 없고,
`pinky_bringup`에는 `cmd_vel` 타임아웃이 없어 마지막 속도가 남는다. 이때는 bringup을 끄거나 로봇 전원 스위치를 쓴다.

## 빌드

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select roport_pinky_line
source install/setup.bash
python3 -m pytest src/roport_pinky_line/test -q
```

`test_node_ros.py`는 `ROS_DOMAIN_ID` 198 + 전용 namespace(`/roport_line_pytest/cmd_vel`)로 발행해 실제 로봇 `/cmd_vel`에 닿지 않는다.

## 실로봇 시험 (사용자 수행)

로봇에서 터미널 2개를 쓴다. pinky_6422는 모든 명령 앞에 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`를 붙인다(이전 사용자 DDS 설정).
멈출 때는 `false`를 보내거나 launch 터미널에서 Ctrl+C. 이상하면 로봇 전원 스위치.

```bash
# 터미널 1 — 모터 (2·3단계에서만)
ros2 launch pinky_bringup bringup_robot.launch.xml
# 터미널 2 — IR 노드 + 라인 추종 (robot 은 기체 이름)
ros2 launch roport_pinky_line line_follow.launch.py robot:=pinky_d660 [dry_run:=true]
# 터미널 3 — 상태 보기 / 켜기·끄기
ros2 topic echo /line_follower/state
ros2 topic pub --once /line_follower/enable std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /line_follower/enable std_msgs/msg/Bool "{data: false}"
```

1. **dry_run (모터 없음)**: bringup 없이 `dry_run:=true`로 띄우고 켠다. 로봇을 손으로 라인 위에서 좌우로 옮기며
   `state`의 `on_line`·`error`·`cmd.angular` 부호를 본다. 라인이 오른쪽(오른쪽 센서가 테이프) → `error` 양수, `angular` 음수.
2. **바퀴 띄움**: bringup을 켜고 `dry_run` 없이 실행한다. 바퀴가 바닥에 닿지 않게 받친 뒤 켠다.
   라인 테이프 조각을 센서 아래(바닥과 같은 높이)에서 좌우로 움직여 바퀴 방향이 맞는지, 테이프를 치우면(`line_lost`) 바퀴가 서는지 본다.
3. **바닥 저속 직선**: `base_speed:=0.03`부터. 직선 1 m, 끝의 가로 테이프에서 `node`로 서는지 확인한다.

조정 인자: `base_speed`, `max_linear`, `max_angular`, `kp`, `kd`, `stop_at_node`(launch 인자). 임계값은 `config/robots.yaml`.
