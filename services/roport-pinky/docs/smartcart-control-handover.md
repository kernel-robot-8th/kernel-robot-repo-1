# 스마트카트 관제 시나리오: 관제 쪽에 필요한 것

- 작성: 2026-11-23, Pinky 담당. 받는 사람: 관제 담당.
- 기준 코드: Pinky = 이 문서가 들어 있는 PR(FFT1-204), 패드 = FFT1-205 PR. 관제 코드 위치는 통합 브랜치 `task/control-integration-test`(7455a89)에서 확인했다.
- 관제 서버 코드는 고치지 않았다. 아래 「제안」은 형식 제안이며 확정 규격이 아니다.
- 실로봇 결과는 2026-11-23 d660(pinky1) 값이다. 관제 경로로 다시 확인한 것은 따로 적었다.

## 1. 요약

목표 시나리오: ARR 도착 → 수령 확인·카드 추종 → 탑승 시간 알림 → Gate 안내 전환 → Gate 도착 → 복귀 → N12 → 라인 → 주차.

| 단계 | 지금 관제 경로 | 필요한 것 | 누가 |
| --- | --- | --- | --- |
| ARR 도착(고객 쪽을 보고 정차) | 됨. 목표 yaw 를 고객 쪽으로 주면 따로 돌 필요 없음 | 없음 | - |
| 수령 확인 + 카드 → 승인 → 추종 | 됨 | 없음 | - |
| 추종 시작 20초 뒤 탑승 시간 알림 | 없음 | 관제 타이머, 로봇 알림 명령(3절), 패드 알림 통로(4절) | 관제, 서비스·패드 |
| 고객 「예」 → Gate 안내 | 없음 | 패드 → 서비스 → 관제 요청, Gate 이동 단계(5절) | 관제, 서비스·패드 |
| Gate 도착 → 복귀 | 복귀 자체는 됨(추종 정지 상태에서) | Gate 단계에서 복귀로 가는 전이 | 관제 |
| N12 복귀 | 2026-11-23 시험에서 실패 | 로봇 실행 환경 변수 하나(2절). 관제 코드 변경 없음 | 로봇 실행 설정 |
| 라인 전환 → 주차 | 됨(H1 → H2 순 자동 배정) | 없음 | - |

## 2. N12 복귀 정밀 접근(관제 코드 변경 없음)

### 실패 원인(2026-11-23 17:54)

- 관제가 보낸 `nav_goal_command`(`goal_id` `N12_RETURN`, `position_tolerance_m` 0.03, `yaw_tolerance_rad` 0.08)가 `nav2_goal_not_reached` 로 실패했다.
- Nav2 는 일반 도착 허용(xy 0.08 m) 안에서 멈추고 성공을 냈다. 드라이버는 그 뒤 지금 자세를 명령의 허용 오차(0.03 m)와 다시 비교해 실패로 처리한다(`trial_driver._drain_results`).
- 허용 오차를 넓히면 N12 라인 전환 조건(5 cm, 가운데 IR 이 선 위)을 못 맞춘다. 그래서 로봇이 마지막 구간을 정밀하게 들어가게 했다.

### 켜는 법

로봇에서 아래 환경 변수를 준다. 값은 정밀 접근을 적용할 `goal_id` 목록(쉼표 구분)이다. 없으면 드라이버 동작은 지금과 같다.

```bash
export ROPORT_FINE_ALIGN_GOALS=N12_RETURN
```

- `trial.launch.py` 를 띄우는 프로세스와 드라이버(`pinky_mqtt.trial_runner`)를 띄우는 프로세스 **둘 다**에 넣는다. 번들 실행 스크립트가 둘을 같은 셸에서 띄우면 한 번만 넣으면 된다.
- 드라이버에만 있고 Nav2 설정에 GateDock 제어기가 없으면 마지막 접근이 `nav2_goal_failed` 로 실패한다. 이 코드는 같은 목적지 재시도 대상이라 관제가 재시도를 반복할 수 있다.
- 게이트 마커 정밀 주차(`ROPORT_GATE_DOCK_CONFIG`)와 같이 쓸 수 없다. 둘 다 있으면 마커 주차가 우선이고 정밀 접근은 켜지지 않는다.

### 동작

1. Nav2 가 목표 20 cm 뒤(목표 yaw 반대 방향)까지 가서 멈춘다.
2. 멈춘 것을 확인한 뒤 목표까지 직선 경로를 느린 제어기(GateDock, 최고 2 cm/s, 충돌 검사 유지)로 간다.
3. 1.5 cm·0.08 rad 안인지 확인한다. 그 뒤 드라이버의 명령 허용 오차 비교(0.03 m·0.08 rad)를 그대로 거친다.

관제가 보내는 좌표·허용 오차는 바꿀 필요가 없다.

### 실패 코드

| 코드 | 뜻 |
| --- | --- |
| `fine_align_timeout` | 마지막 접근이 25초 안에 안 끝남 |
| `fine_align_outside_envelope` | 접근 시작 위치가 목표에서 0.30 m 또는 0.35 rad 넘게 벗어남 |
| `fine_align_no_goal` | 내부 오류(명령 없음) |
| `gate_dock_stop_unconfirmed` | 20 cm 뒤에서 3초 안에 정지 확인 안 됨 |
| `gate_dock_controller_unavailable` | GateDock 제어기 액션 서버 없음 |
| `gate_dock_final_tolerance` | 마지막 확인(1.5 cm·0.08 rad) 실패 |

- 이 코드들은 같은 목적지 재시도 대상(`NAV_RETRY_FAULTS`)이 아니다. 실패하면 로봇은 정지 상태(halted)로 남는다. 관제에서 재시도를 허용할지는 정하지 않았다(8절).
- 소요 시간: 일반 이동 뒤 정지 확인 최대 3초 + 마지막 접근 최대 25초가 더 든다. `command_timeout_s` 가 이를 덮는지 확인이 필요하다.

### 실로봇 근거와 한계

- 20 cm 접근이 25초 시간 초과로 실패하다가, GateDock 속도 설정 3개를 바꾼 뒤 19초에 성공했다(1회, 이어서 라인 전환·주차까지 성공). 이 PR 에 그 설정이 들어 있다.
- 새 속도는 정밀 접근으로 띄울 때만 들어간다. 게이트 마커 주차(`ROPORT_GATE_DOCK_CONFIG`)로 띄우면 변경 전 값을 쓴다.
- 이 결과는 시험 실행기 경로다. 관제 경로(`N12_RETURN` + 위 환경 변수)는 실로봇에서 아직 확인하지 않았다.
- 제한 시간 25초 대비 여유가 6초다. 더 돌려 봐야 한다.

## 3. 탑승 시간 알림: 로봇 쪽

### 드라이버 hook(이 PR 에 구현)

`driver.set_boarding_alert(on)` (`on` 은 bool, 아니면 `ValueError('invalid_alert')`).

- 진행 중인 명령(추종 중 포함)과 상관없이 부를 수 있다. `validate_action`·`dispatch` 를 거치지 않는다.
- 켜져 있는 동안 로봇 표시가 바뀐다: LCD 「탑승 시간입니다」, LED 주황 점멸, 부저 5초마다. 로봇에 `status_indicator`(`smartcart:=true`)가 켜져 있어야 한다.
- 표시는 멈춤·오류 상태(BLOCKED, PAUSED, UNAVAILABLE, STOPPED, ERROR)가 우선이다.
- 스스로 꺼지는 조건: runner 가 `heartbeat()` 를 1초 넘게 부르지 않을 때, 드라이버 `close()`. `safe_stop` 으로는 꺼지지 않는다.
- 스스로 꺼진 뒤 다시 켜려면 `set_boarding_alert(True)` 를 다시 불러야 한다.
- `observe()` 에는 알림 상태가 없다(기존 필드 그대로).
- 추종 명령이 살아 있는 동안 `observe()` 에 선택 필드 `follow = {active, follow_mode: "marker"[, follow_session_id]}` 가 붙는다. `active` 는 Nav2 의 추종 goal 수락과 신선한 `trial_guard` 응답의 owner `nav2` 를 모두 확인했을 때만 true 다(자세한 조건은 `roport_pinky` README).

### MQTT 명령 형식 제안

지금 계약에는 알림 명령이 없다. 모르는 type 은 번들이 경고 로그만 남기고 `command_status` 를 보내지 않는다.

```json
{
  "type": "boarding_alert_command",
  "payload": {"expected_boot_version": "<boot id>", "on": true}
}
```

- 번들 처리 제안: `robot_stop_command` 처럼 활성 명령 검사 밖에서 받아 `driver.set_boarding_alert(on)` 을 부르고, 결과를 `command_status` `completed`(또는 `rejected` + `invalid_alert`)로 돌려준다. 추종 명령이 진행 중이어도 받아야 한다.
- 손댈 곳(통합 브랜치 기준): `adapters/mqtt/field_wire.py`(COMMANDS, SCHEMAS, 대상 검사), `interfaces/mqtt/roport-mqtt.yaml`, `interfaces/mqtt/examples.json`, `adapters/mqtt/field_pinky.py`, `executor/fleet.py` 의 `command_resources`, 관련 테스트.

### 관제에서 켜고 끄는 시점 제안

- 켜기: 추종 시작 20초 뒤(시험값). 패드 문구는 「탑승 시작 15분 전」으로 고정이다. 패드는 카트가 주행 중이라고 보고된 동안에 온 알림만 창으로 띄우므로(출발 확인 전·일시정지 중에는 무시), 알림은 추종이 진행 중일 때 보낸다. `following` 단계에 들어간 시각을 지금은 저장하지 않으므로(`fleet.py` 1021-1026줄 부근) 그 시각을 남겨야 한다. 1초 주기 블록(`fleet_runtime.py` 265-297줄 부근)에 걸 수 있다.
- 끄기: Gate 안내 전환을 받았을 때, 추종을 정지·종료할 때, 복귀를 시작할 때.

## 4. 탑승 시간 알림: 패드 쪽

- 패드(FFT1-205)의 「추종 중」 화면에 알림 창을 넣었다. 문구 「탑승 시작 15분 전입니다」, 버튼 「예, 게이트 안내로 전환」·「아니오, 계속 따라오게 할게요」.
- 패드는 스스로 시간을 재지 않는다. 알림 보고가 들어올 때만 창을 띄운다. 지금은 데모 모드의 모의 입력 버튼으로만 뜬다.
- 서버 모드에서 창이 뜨려면 두 가지가 필요하다(서비스·패드 담당과 협의).
  1. 관제 → 패드 알림 통로. 패드에는 상태 조회·폴링이 없다. 관제 진행 상태는 서비스 서버 `control_task_views` 까지 가지만 고객 웹만 읽는다.
  2. 서버 모드의 「추종 중」 화면. 통합 브랜치 패드는 수령 확인 뒤 「복귀 중」 화면으로 넘어가 「추종 중」 화면을 쓰지 않는다.
- 창이 떠 있는 동안 패드의 큰 「중지」 버튼은 가려진다. 고객은 「아니오」로 닫은 뒤 눌러야 한다. 출발·재개 직후에 창이 뜨면 패드에서 바로 멈출 수단이 없으므로 그때 정지는 관제 화면에서 한다.

## 5. Gate 안내 단계 제안

- 로봇 쪽 변경은 필요 없다. 기존 `nav_goal_command` 로 Gate 좌표에 간다.
- 패드 「예」는 기존 「게이트 안내로 전환」과 같은 요청(`requestService(GATE_GUIDE)`)이다. 서버 모드에서는 이 요청과 중지·재개가 막혀 있고, 서비스 서버 cart API 와 서비스 ↔ 관제 메시지에 대응 항목이 없다.
- 관제 단계 제안: `following` → (Gate 안내 요청) → 추종 정지 확인 → 알림 끄기 → Gate 로 `nav_goal_command` → 도착 대기 → `return_nav`.

Gate 좌표 후보(`roport_smartcart/config/places_sim.yaml`, map 좌표, yaw −3.1398):

| Gate | x | y |
| --- | --- | --- |
| G1 | 0.1129 | −0.8106 |
| G2 | 0.1122 | −0.4316 |
| G3 | 0.1114 | −0.0216 |

- 시험 실행기로 추종 위치 → G3 이동 1회 성공(19초, 허용 0.3 m). Gate 에서 N12 로 돌아올 때는 입구 가운데 경유점 `(2.60, −1.284)` 를 거쳤다. 직행하면 03 아래 벽 끝을 바짝 돈다.
- 관제 경로에서 Gate 도착 허용 오차를 어떻게 줄지는 정하지 않았다.

## 6. 로봇 실행 설정

- `ROPORT_FINE_ALIGN_GOALS=N12_RETURN` (2절).
- Nav2 설정에 GateDock 제어기가 있어야 한다. `trial.launch.py` 는 위 환경 변수가 있으면 넣는다. `config/nav2_follow_real.yaml` 에는 제어기가 이미 있지만 정밀 접근 속도는 없다. 속도는 환경 변수가 있을 때 `trial.launch.py` 가 넣으므로 launch 쪽에도 꼭 넣는다.
- 표시: `ros2 run pinky_led led_server`, `ros2 run roport_line_nav status_indicator --ros-args -p smartcart:=true`.
- 노트북 인식 도구(`roport_follow tool`)가 같은 ROS 도메인에서 켜져 있어야 한다. 관제는 놓침 기준을 보내지 않으므로 노트북도 기본값(1초)으로 켠다.

## 7. 운영 주의(2026-11-23 실로봇에서 겪은 것)

- 드라이버(runner)만 다시 켜면 구동권 lease 가 끊겨 `drive_lease_expired` 로 잠긴다. `trial.launch.py` 도 같이 다시 켠다.
- 주행 중에 로봇이나 같은 도메인의 PC 에서 ROS 프로그램을 켜고 끄지 않는다. 구동권 감시가 끊겨 `drive_guard_unavailable` 로 잠긴 적이 있다(원인은 확정하지 못했다).
- `trial.launch.py` 를 다시 켠 뒤 노트북 인식 도구의 영상 지연 기준이 어긋나 인형 장애물 정보가 끊길 수 있다. 로봇은 「Costmap timed out」으로 출발을 못 한다. 드라이버를 켜기 전에 노트북 도구를 다시 켜면 풀린다.
- `trial.launch.py` 를 켠 뒤 약 60초 안에 처음 위치를 넣어야 Nav2 가 올라온다.

## 8. 미확인·미정

- 관제 경로에서의 정밀 접근(`N12_RETURN` + 환경 변수) 실로봇 동작.
- 드라이버가 낸 탑승 알림으로 LCD·LED·부저가 켜지는지(시험 실행기 경로는 hook 으로 바꾼 뒤 실로봇에서 다시 돌리지 않았다).
- 게이트 마커 정밀 주차와 정밀 접근을 한 실행에서 같이 쓰는 구성(지금은 마커 주차가 우선이라 N12 정밀 접근이 꺼진다).
- `fine_align_*`·`gate_dock_*` 실패 뒤 관제의 재시도·복구 절차.
- 탑승 알림 명령 형식, Gate 안내 요청 형식, 관제 → 패드 알림 통로.
