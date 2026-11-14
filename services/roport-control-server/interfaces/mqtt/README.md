# 현장 MQTT 계약

현재 로컬 구현은 `schema_version=2.0`, `profile=roport-control-v2`다. PR #68의 AsyncAPI 초안에서 토픽의 `v1/{site}`와 `route.requested` 표현을 제거하고, 실행 중복 방지에 필요한 `command_id`와 기동 구분 `boot_version`을 유지했다. 아직 병합·실물 검증 완료를 뜻하지 않는다.

- [AsyncAPI](roport-mqtt.yaml): 전체 12종의 토픽·JSON 구조.
- [JSON 예시](examples.json): 조회, 실행, 결과, 재시작 방어, 출발 승인, 양방향 N12 전환.
- [실행 검증 코드](../../src/roport_control_server/adapters/mqtt/field_wire.py): 명령 대상, 상관관계, 필드 조합까지 검사하는 정본. 문서와 코드의 일치는 테스트로 확인한다.
- [Pinky 이식·구동 경계](../../docs/mqtt-field.md).

Pinky와 OMX v2 Host에 같은 계약을 적용한다. OMX 단독·공동 작업은 실제 MQTT와 가상 장치로 검증하며, 현장 장치·센서 연결은 별도다. [OMX 적용 조건](../../../roport-omx/README.md#최소화-규약-현장-host-fft1-180)을 확인한다. 기존 OMX 시험은 기존 계약을 유지한다.

| 토픽 | 메시지 | 방향 / QoS |
| --- | --- | --- |
| `roport/robot/{robot}/cmd` | `robot_state_request`, `line_route_command`, `set_drive_mode_command`, `nav_goal_command`, `arm_action_command`, `warehouse_load_command`, `robot_stop_command` | 관제 → 로봇 / 1 |
| `roport/workcell/shelf-01/cmd` | `robot_state_request`, `shelf_transfer_command`, `robot_stop_command` | 관제 → 공동 작업셀 / 1 |
| `roport/robot/{robot}/event`, `roport/workcell/shelf-01/event` | `robot_state`, `command_status` | 로봇/작업셀 → 관제 / 1 |
| `roport/robot/{robot}/ack`, `roport/workcell/shelf-01/ack` | `event_stored` | 관제 → 발행 주체 / 1 |
| `roport/robot/{pinky}/motion` | `robot_pose` | Pinky → 표시 UI / 0 |

모든 발행은 `retain=false`. 물리 명령에는 만료 시각이 필요하다. QoS1 PUBACK은 브로커 수신 확인이며 동작 완료나 관제 저장 확인이 아니다. 중요 결과는 저장 후 `event_stored`를 받아야 outbox에서 제거한다.

`message_id`는 전송 메시지, `command_id`는 한 실행을 구분한다. 같은 명령의 재전송은 두 값과 본문 전체를 유지한다. `boot_version`은 프로그램 재기동마다 새로 생성하며 파일 버전이 아니다. `robot_state`의 `sequence`는 기동 내 상태 순서, `command_status.sequence`는 **해당 command_id 안의 상태 순서**, `robot_pose.sequence`는 표시 샘플 순서다.

상태 조회 `{}`는 모터 동작을 유지하지 않는다. 활성 명령을 유지하려면 같은 `expected_boot_version`과 `keep_running_command_id`를 새 메시지로 주기 전송한다. Pinky에서 3초 동안 유효한 유지 확인이 없으면 정지/HOLD한다. ROS drive guard의 1초 lease는 별도로 유지한다.

관제 내부 주문·계획·작업 키와 구형 trial journal 표현은 MQTT에 보내지 않는다. `goal_id`는 내부 목표 이름을 전달하는 선택 필드이며 이 구현에 마커 주차 동작은 포함하지 않는다. 도착 표시용 `robot_pose`만으로 구역을 해제하거나 적재 완료를 확정하지 않는다.

모델·도킹 캘리브레이션, 실제 통로 폭과 팔 간섭, 하드웨어 정지 시간은 현장 검증 대상이다. 가상 Nav2는 장애물 없는 위치 이동 모델이며 ROS Nav2 경로 계획 검증을 대신하지 않는다.

## 시각 동기화 전제 (미정)

수신한 로봇은 **자기 시계의 현재 시각**으로 `sent_at`과 `expires_at`을 판정한다(Pinky: `field_pinky.py`의 `_fresh`). 먼저 `sent_at`이 로봇 시계 기준 −0.5~+3초 창 안인지 보고 벗어나면 `stale_message`로 거절하며, 그 다음 `expires_at`이 지났으면 `expired_command`로 거절한다. 그래서 로봇 시계가 이 창보다 크게 어긋나면(0.5초 넘게 늦거나 3초 넘게 빠르면) **모든 명령이 `stale_message`로 거절**된다. 안전한 쪽으로 실패하지만 시험이 되지 않는다. 만료된 명령이 유효로 판정될 수 있는 경우는 이 창 안의 어긋남(약 0.5초 이내)으로 좁혀진다. 관제의 `field_wire.py`는 `sent_at`보다 `expires_at`이 늦은지만 검사하고 시각 동기화를 요구하지 않는다. 현장망에서 Pinky 시계가 크게 어긋난 사례가 있었다고 한다(Pinky 쪽 리뷰 보고, 직접 확인하지 못함). 이 경우는 "만료 명령 수락"이 아니라 "전부 거절"로 나타난다. 코드를 현장에 적용하기 전에 관제·로봇 시각 동기화(NTP)와 허용 오차를 정해야 한다.

## 검증과 보기

Node.js가 필요하다. `@asyncapi/cli@3`은 HTML 템플릿과 버전이 맞지 않아 실패하므로 `@latest`를 쓴다.

```bash
# 문법 검증
npx @asyncapi/cli@latest validate services/roport-control-server/interfaces/mqtt/roport-mqtt.yaml
# 편집·미리보기 (브라우저)
npx @asyncapi/cli@latest start studio services/roport-control-server/interfaces/mqtt/roport-mqtt.yaml
# 단일 HTML 생성 (저장소 밖 폴더에 만든다)
npx @asyncapi/cli@latest generate fromTemplate services/roport-control-server/interfaces/mqtt/roport-mqtt.yaml @asyncapi/html-template@3 -o /tmp/mqtt-doc --force-write --param singleFile=true --param outFilename=index.html
```

생성한 HTML은 저장소에 올리지 않는다. `.gitignore`에 `out/`이 없으므로 출력 폴더는 저장소 밖(`/tmp/...`)으로 잡는다. 필요할 때 다시 만든다.

## 수정 규칙

- yaml과 `field_wire.py`는 함께 바꾸고 문서와 코드의 일치는 테스트로 확인한다. 토픽·필드를 바꾸면 관제·Pinky·OMX 담당자가 함께 확인한다.
- 예시 값은 가짜 값만 쓴다. 실제 SSID·IP·인증정보·현장 좌표를 넣지 않는다([공개 저장소 보안 규칙](../../../../.agents/rules/public-repository-security.md)).
