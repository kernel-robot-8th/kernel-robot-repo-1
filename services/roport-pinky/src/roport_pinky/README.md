# roport_pinky

Pinky 연동 패키지. 지금은 관제 **trial-v1 주행 시험**(Confluence 19988482 「Pinky 주행 시험 — 관제 구현 및 MQTT 이식 가이드」)의 Pinky 쪽 드라이버만 있다.

## trial_driver

관제 키트 `pinky-mqtt-trial-v1-scenarios.zip`(SHA-256 `7dd2a749…0ece1c`)의 `pinky_mqtt.trial_runner`가 `roport_pinky.trial_driver:create_driver`를 불러 쓴다.
MQTT·메시지 검사·이력·재전송은 키트가 맡고, 이 드라이버는 `roport_line_nav`와 이어 준다. 키트 코드는 이 저장소에 넣지 않는다(관제 담당 로컬 구현, 첨부로만 배포).

| hook | 동작 |
|---|---|
| `observe()` | `line_nav/state`(1 s 이내)·`odom`(0.5 s 이내)으로 현재 노드·정지·라인 사용·방향(map rad = layout + 0.1°)을 돌려줌. 상태가 끊기면 `fault=line_nav_unavailable` |
| `validate_action()` | `route.requested`만, `start_maneuver=none`만. 경로(`plan_route`)·그래프 버전·출발 방향(첫 구간과 0.35 rad 이내)·속도 하한·line_nav 주행 중 여부 검사 |
| `dispatch()` | `line_nav/command`로 `speed`(= min(현재 속도, `speed_limit_mps`)) → `start`. 상차 대기 중이면 `load_done`을 먼저 보내고 진입선으로 돈 뒤 출발 |
| `drain_results()` | 노드마다 `progress`(seq·prev·next), 목표 노드에서 정지하면 `completed`, line_nav 오류·정지면 `failed`(사유 첫 단어) |
| `safe_stop()` | `line_nav/command` stop + `cmd_vel` 0 |

- 출발 노드: 처음에는 환경변수 `ROPORT_PINKY_START_NODE`(운영자가 놓은 칸, 예: P5 = `N16`). 칸 노드는 칸 바깥 방향으로 놓았다고 본다. 그 뒤로는 line_nav가 확인한 마지막 노드. 정지·오류 뒤에는 모름이 되어 관제의 다음 route가 거부된다(드라이버를 다시 시작하고 출발 노드를 다시 지정).
- 도착: N01·N11(상차)은 반시계 90° 뒤 상차 대기(`loading`)에서 `completed`(사용자 결정 2026-11-05). 대기·충전 칸은 후진 정차 뒤.
- 지원 안 함: `mode_switch`·`navigate`(Nav2), `turn_back`·`back_out`, 경로 중간 정차 노드.
- `map_id`·`map_version`은 trial 설정 값을 그대로 보고한다(라인 주행은 지도를 쓰지 않음).

## 실행 (로봇, 사용자 수행)

순서: `pinky_bringup` → `roport_line_nav`(guard 포함) → 이 드라이버. 모두 같은 `ROS_DOMAIN_ID`.

```bash
# 키트: ~/roport_trial/kit (ZIP 압축 해제), 설정: 관제와 같은 trial.json
python3 -m venv --system-site-packages ~/roport_trial/venv && ~/roport_trial/venv/bin/pip install paho-mqtt==2.1.0
source ~/pinky_test_ws/install/setup.bash
export TRIAL_DIR=~/roport_trial/kit/scenarios/single/hardware
source $TRIAL_DIR/broker.env               # ROPORT_MQTT_HOST = 브로커 PC 주소
export ROPORT_PINKY_START_NODE=N16          # P5 칸에 칸 바깥을 보고 놓음
cd ~/roport_trial/kit && ~/roport_trial/venv/bin/python -m pinky_mqtt.trial_runner \
  --config $TRIAL_DIR/trial.json --role logistics --journal ~/roport_trial/logistics.json \
  --driver roport_pinky.trial_driver:create_driver --hardware-authorized
```

관제가 보낼 값(P5 → P7, `to_delivery`): `start_heading_rad = -1.5690`(남쪽 −π/2 + 0.1°), `start_maneuver = none`, `speed_limit_mps` 0.03~0.06(실로봇 확인 0.03).

## 테스트

`python3 -m pytest src/roport_pinky/test` — RouteExecutor + 2D 시뮬레이터로 판정부만 확인한다(ROS·MQTT·실로봇 아님).
