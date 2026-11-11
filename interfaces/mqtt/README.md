# MQTT 인터페이스 (AsyncAPI)

관제 ↔ Pinky / OMX 작업셀 MQTT 계약을 [AsyncAPI](https://www.asyncapi.com/) 3.0으로 기록한다. 기준은 Confluence "Interface Specification (comm)"의 운영 검토안이다. 원문을 복제하지 않고 토픽·필드·QoS 구조만 담는다.

## 상태: 설계 초안

- 현재 `route.requested`, `route.progress` 2개 메시지만 있다. 나머지 메시지(`dock`, `depart`, `navigate`, `stop/resume/cancel`, `notify`, `load`, `shelf`, `ack`, `telemetry`, `motion`, `presence`, `heartbeat`)는 합의되는 대로 추가한다.
- **현재 코드와 일치하지 않는다.** 이 문서는 필수 ID를 `message_id`, `robot_id`, `user_id`, `product_id`, `gate_id` 5개로 줄이는 설계를 담고 있다. 관제 서버 `adapters/mqtt/messages.py`와 OMX `mqtt_link/`는 아직 `task_id`, `step_id`, `command_id`, `execution_id`, `site_id`를 필수 헤더로 요구하고 `boot_id`·`command_id`를 재시작 방어·중복 실행 방지에 쓴다. 코드 적용은 별도 작업이며 이 문서가 구현을 뜻하지 않는다.
- 시험용 스키마 `roport-pinky-driving-trial-v1`은 포함하지 않는다.
- `follow.requested`(고객 추종)는 시나리오 확정 전이라 넣지 않았다.

## 필드 규칙

- 필수 ID는 `message_id`, `robot_id`, `user_id`, `product_id`, `gate_id`뿐이다.
- `task_id`, `step_id`, `command_id`, `execution_id`, `site_id`는 필요한 정보이지만 메시지 필드에는 싣지 않는다. 환경(`site`)은 토픽 주소의 `{site}`로 구분한다. 이 정보를 어디서 관리하는지는 별도로 정한다.
- 상태는 enum 필드로 표현한다.
- 메시지 종류별로 필요한 ID만 필수로 둔다. 토픽(`robot/{robot_id}/…`)에 이미 있는 `robot_id`는 payload에 중복해 넣지 않는다.
- 응답(event·ack)은 `in_reply_to`에 명령의 `message_id`를 넣는다.

## 검증과 보기

Node.js가 필요하다. `@asyncapi/cli@3`은 HTML 템플릿과 버전이 맞지 않아 실패하므로 `@latest`를 쓴다.

```bash
# 문법 검증
npx @asyncapi/cli@latest validate interfaces/mqtt/roport-mqtt.yaml
# 편집·미리보기 (브라우저)
npx @asyncapi/cli@latest start studio interfaces/mqtt/roport-mqtt.yaml
# 단일 HTML 생성 (커밋하지 않는다)
npx @asyncapi/cli@latest generate fromTemplate interfaces/mqtt/roport-mqtt.yaml @asyncapi/html-template@3 -o out --force-write --param singleFile=true --param outFilename=index.html
```

생성한 HTML과 `out/`은 저장소에 올리지 않는다. 필요할 때 다시 만든다.

## 수정 규칙

- yaml이 정본이다. 토픽·필드를 바꾸면 이 파일을 PR로 고치고 관제·Pinky·OMX 담당자가 함께 확인한다.
- 예시 값은 가짜 값만 쓴다. 실제 SSID·IP·인증정보·현장 좌표를 넣지 않는다([공개 저장소 보안 규칙](../../.agents/rules/public-repository-security.md)).
