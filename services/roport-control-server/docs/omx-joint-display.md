# OMX 관절 표시 연동과 2A 샘플

OMX 담당자는 2A 단독 작업에서 **왼팔의 한 관측 시점**을 골라, 패널이 읽은 값 6개와 Host가 그 값으로 만든 MQTT 메시지를 한 파일에 담는다. 아래 명령은 저장소 루트에서 실행한다.

```bash
uv run --directory services/roport-control-server python tools/check_omx_2a_sample.py docs/examples/omx-2a-sample.json
```

예제의 임의 관절값과 고정 시각, 영상 파일명은 제출 형식을 보여 주는 합성 자료다. 현장 파일은 마지막 인자를 캡처의 절대 경로로 바꿔 검사한다. 검사는 브로커나 로봇에 접속하지 않는다. `OK`는 전송 형식·관절 구성·관측 시각·팔 식별자가 일치한다는 뜻이다. 보정 정확도는 실물과 지도 자세를 대조해서 판정한다.

## 관제 측 전송 규격

| 항목 | 합의한 값 |
| --- | --- |
| 메시지 | `schema_version: "2.0"`, `profile: "roport-control-v2"`, `type: "omx_joint_state"`. `source`는 팔 ID, `target`은 양측 설정의 `control_id`다. 메시지마다 고유한 `message_id`와 시간대가 있는 `sent_at`을 넣는다. |
| 채널 | 팔마다 `roport/robot/omx1/motion` 또는 `roport/robot/omx2/motion`. QoS 0, retain=false. 유실된 표시 프레임은 건너뛴다. |
| 관측 | `payload.boot_version`은 해당 팔의 v2 `robot_state`가 보고한 기동값이다. `sequence`는 팔별·기동별로 관측마다 증가한다. `observed_at`은 패널이 원본 값을 읽은 시각이며 시간대를 포함한다. 같은 관측을 다시 읽으면 처음 시각을 유지하고 재발행을 생략한다. |
| 주기 | `sample_kind: "mission"`은 제어 루프의 최신 관측을 최대 10 Hz로 보낸다. 1초 넘게 새 관측이 없으면 Host는 발행을 멈춘다. `"idle"`은 기존 준비 상태 측정(약 10초 간격)이 새로 들어왔을 때 한 번 보낸다. |
| 자세 | `payload.joints`에 아래 6개를 모두 유한한 rad 값으로 보낸다. 팔의 설치 위치·방향은 지도 소유다. 위치나 속도를 채울 필요가 없다. |

| 패널 정규화 관절 | shoulder_pan | shoulder_lift | elbow_flex | wrist_flex | wrist_roll | gripper |
| --- | --- | --- | --- | --- | --- | --- |
| URDF 관절 | joint1 | joint2 | joint3 | joint4 | joint5 | gripper_joint_1 |

Host가 보정표의 `panel_grid → q_of_panel`로 패널 값(팔 −100~100, 그리퍼 0~100)을 변환한다. `gripper_joint_2`는 URDF에서 첫 손가락 회전을 −1배로 따라간다. 패널의 기존 관측을 재사용하고 변환·발행은 30 Hz 제어 루프 밖에서 처리한다. 정렬·복귀에서 새 관측원이 없으면 마지막 관측이 그대로 오래되도록 둔다.

Host의 표시 송신 작업에서 아래 두 함수를 사용한다. `payload`에는 표의 관측 필드 5개를 담는다. 표시 연동을 끄거나 송신에 실패해도 기존 미션 루프는 계속 돈다.

```python
from roport_omx.mqtt_link import field_wire

try:
    message = field_wire.build_message("omx_joint_state", arm_id, control_id, payload)
    transport.publish_motion(field_wire.topic(arm_id, "motion"), message)
except (ValueError, ConnectionError):
    pass  # 이 관측만 건너뛰고 다음 측정을 기다린다.
```

처음 관측하기 전에는 지도 기본 자세를 쓴다. 미션은 마지막 측정 후 2초, 대기는 20초를 넘기면 관제에서 마지막 측정 자세를 유지하며 지연을 표시한다. 같은 기동의 역순·중복 관측, 1초보다 먼 미래 시각, 이미 지연 기준을 넘긴 패킷은 버린다. 새 기동은 해당 팔의 v2 상태로 확인한다. 작업 단계·진행률·완료는 기존 관제 작업 상태를 따른다. 표시 스트림만 유실되면 작업은 계속된다. 관제 브로커 전체가 끊겼을 때는 기존 watchdog/HOLD 조건을 따른다.

## 2A 실측 인계

1. **2A의 왼팔을 식별한다.** 패널 `left` → `omx1`, 관제 슬롯은 2A다. 2C는 `right` → `omx2`, 2B는 양팔이다. 단독 작업의 미션 샘플에는 참여한 팔만 기록한다. 선반 정면과 2A 표지, 움직인 팔을 함께 식별할 수 있는 사진 한 장 또는 짧은 영상에 촬영 방향과 현장에서 확인한 도킹 노드를 적는다. 지도상의 좌우·설치 방향은 이 기준으로 대조한다.
2. **같은 프레임으로 묶는다.** [예제](examples/omx-2a-sample.json)를 복사해 `sample_origin`을 `measured`로 바꾸고, 실제 `handover_fk.json`의 버전 또는 SHA-256과 패널 원본 6개, 실제 송신한 토픽과 JSON을 넣는다. 같은 `observed_at`의 사진 또는 영상 시점과 동작 단계(예: 집기 직전)를 함께 적는다. Host의 송신 지연은 `sent_at − observed_at`로 확인한다. 수집한 파일들은 위 명령에 공백으로 나열해 검사할 수 있다.
3. **자세와 주기를 확인한다.** 먼저 2A 미션 한 프레임에서 왼팔에 해당 미션의 자세가 반영되고 오른팔에는 같은 값이 복제되지 않는지 본다. 오른팔에서 별도 대기 관측이 들어올 수 있다. 가능하면 홈·앞으로 뻗기·손목 세우기·그리퍼 열기에서 같은 형식으로 더 남긴다. 실물과 지도의 영점·회전 방향·그리퍼 열림을 대조한다. 연속 프레임 구간에서는 `sequence` 간격과 `observed_at` 간격으로 10 Hz 송신과 누락을 확인한다. 실측 파일·촬영본은 팀 내부 공유 경로로 전달하고 PR에는 합성 예제만 둔다.

2A 비교는 단일 팔의 기하 검증이다. 양팔 성능은 동일 조건의 2B 평가에서 표시 켬/끔 각 5회로 비교한다. 30 Hz 루프 지연, 카메라 fps·age, 추론 RTT·timeout, 미션 성공 여부를 기존 패널 로그로 확인한다.

인계 파일의 `evidence`에는 현장에서 읽은 도킹 노드, 촬영 방향, 동작 단계, 팀 내부 촬영본 참조를 기록한다. 동영상은 해당 관측 장면까지의 시간(ms)을 `clip_offset_ms`에 넣고 정지 사진은 이 값을 생략한다. MQTT 발행 대상은 `wire` 객체 한 개다.

## 합성 샘플로 3D 화면까지 확인하기

Mosquitto가 설치된 개발 PC에서는 아래 한 줄로 같은 합성 관절값이 OMX Host SDK에서 로컬 브로커와 관제 소켓을 거쳐 WebSocket에 도착하는지 확인할 수 있다. 시험은 임시 브로커를 루프백에 띄우고 샘플의 기동값과 관측 시각을 시험 실행에 맞춘다.

```bash
REQUIRE_MQTT_TESTS=1 uv run --directory services/roport-control-server pytest -q tests/test_omx_joints.py
```

Python 3.12, uv, Node.js 24가 있는 개발 PC에서 저장소 루트를 기준으로 아래 명령을 실행한다. 로컬 브라우저 시험은 합성 스트림과 별도 시험용 관제 실행부를 사용한다.

```bash
cd services/roport-control-server
uv sync --locked
npm ci --ignore-scripts
npm run prepare:assets
npx playwright install chromium
npx playwright test tests/browser/fleet.spec.mjs -g 'OMX 2A'
```
