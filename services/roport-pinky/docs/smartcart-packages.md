# 스마트카트 기체별 실행 패키지

스마트카트 기능은 기체에 따라 서로 다른 패키지로 구현돼 있다. 관제가 어떤 기체를 스마트카트로 배정하느냐에 따라 로봇에서 실행되는 코드와 받을 수 있는 명령이 달라진다.

| 기체 | 관제 로봇 ID | 스마트카트 패키지 | 하는 일 |
| --- | --- | --- | --- |
| `pinky_d660` | `pinky1` | `roport_pinky`(`trial_follow`) + `roport_follow` | 고객 카드(ArUco 35~49) 추종, 추종 중 인형 우회 |
| `pinky_764e` | `pinky3` | `roport_smartcart` | 라인 상차 → 대기 복귀 → 관제 호출 → Nav2 배송 → 하차 확인 → 복귀 |

## `pinky_d660`: `roport_pinky` + `roport_follow`

- 로봇: `ros2 launch roport_pinky trial.launch.py ... params_file:=<share/roport_pinky/config/nav2_follow_real.yaml>`
- 노트북(같은 ROS_DOMAIN_ID): `roport_follow`의 `follow_tool`이 로봇 카메라 영상에서 카드·인형을 인식해 `follow/target`·`follow/status`·`/doll_obstacles`를 보낸다. 노트북이 없으면 추종할 수 없다.
- 드라이버 명령: `follow.requested` `{card_id, standoff_m, ...}` → 결과 `follow.failed`의 `error_code`. 상세는 [roport_pinky README](../src/roport_pinky/README.md)의 「고객 카드 추종」 절.
- 관제 MQTT(`roport-control-v2`)에는 아직 추종 명령이 없다. 지금은 로봇 안 시험 도구(`roport_follow`의 `demo_runner`)가 드라이버를 직접 부른다.

## `pinky_764e`: `roport_smartcart`

- 실행·명령·검증 범위는 [roport_smartcart README](../src/roport_smartcart/README.md)가 기준이다(실물 복귀 스택 `real_return_stack.launch.py`, 관제 trial-v1 역할 smartcart).
- 고객 카드 추종(`follow.requested`)은 이 패키지에 없다.

## 관제에서 주의할 점

- 두 패키지를 한 기체에서 동시에 실행하지 않는다(둘 다 Nav2 와 `trial.launch.py` 설정을 쓴다).
- 기체를 바꿔 스마트카트로 배정하려면 그 기체에 해당 패키지가 설치돼 있어야 한다. `pinky_d660`은 물류(`roport_line_nav`)와 겸용이다.
