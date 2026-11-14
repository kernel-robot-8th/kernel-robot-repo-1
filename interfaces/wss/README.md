# 서비스 서버 ↔ 관제 서버 WSS 계약 (S1 제안)

**상태: 팀 합의 전 제안이다.** 양쪽 모두 아직 구현하지 않았고 Jira FFT1-183에서 합의한다. 이 문서의 type·필드·규칙은 합의 뒤에 바뀔 수 있다. 시험이 통과해도 실장비·실서버 연동 검증을 뜻하지 않는다.

- [AsyncAPI](roport-wss.yaml): S1 type 5종의 envelope·payload 스키마.
- [JSON 예시](examples.json): 세션 시작, 요청, 저장 확인, 거부, 충돌, 전송 거부와 `request_sha256` 벡터.
- 예시 검증: `services/roport-control-server/tests/test_wss_contract_examples.py`. 서비스 서버와 관제 코덱은 같은 예시를 재사용한다(아직 없음).

## 범위 (S1)

연결·인증·envelope 검증·`message.ack`·outbox/inbox·재연결 때 미확인 재전송까지와 `task.requested → task.rejected` 한 흐름만 다룬다. 관제에는 운영 task 모델이 없으므로 S1의 관제는 요청을 **항상 거부**하며 `task.accepted`를 보내지 않는다. 실예약(`PickupService`)에는 연결하지 않고 기능 플래그(`service.wss.enabled`)는 꺼진 채 격리 DB에서만 시험한다.

| 후속 | 내용 |
| --- | --- |
| S2 | `task.accepted`·`progress`·`completed`·`failed`, `control_task_views`, UNKNOWN 관리자 대조 |
| S3 | durable cursor, `sync.request/snapshot/completed`, stale 표시 |
| S4 | `task.change_requested` |
| S5 | `handover.*` (직접 수령 삭제 반영 후 재정의) |

## 연결과 인증

- 관제가 `wss://{service-host}/ws/v1/sites/{site_id}/control`로 연결을 시작한다. 서비스는 현장으로 연결을 열지 않는다. 브라우저 ↔ 관제 API의 `/api/state` WebSocket과는 별개다.
- 인증은 HTTP upgrade 요청의 `Authorization: Bearer` 헤더이며 site당 하나의 자격이다. URL query의 토큰은 거부한다. 서버에는 SHA-256 해시만 두고 값은 저장소·로그·오류 메시지에 넣지 않는다. 기존 `/api/**` 인터셉터는 WebSocket 핸드셰이크에 적용되지 않으므로 핸드셰이크 전용 검사를 둔다.
- `site_id`는 `^[A-Za-z0-9_-]{1,128}$`다. 경로에 `%`, `;`, `//`, dot segment, 끝 슬래시가 있으면 101 없이 거절한다.
- site당 활성 연결은 1개다. 새 연결이 upgrade에 성공하면 연결 세대를 올리고 이전 세션을 close 4001로 닫는다. 관제는 4001을 받으면 자동 재접속하지 않는다.
- 운영은 TLS(reverse proxy 종단)만 쓴다. 평문 `ws://`는 loopback이고 명시 플래그가 있을 때만 허용한다.

## 신뢰성 규칙 요약

- 업무 변경과 outbox insert, 수신 inbox insert와 업무 반영과 응답 outbox insert는 각각 한 트랜잭션이다. `message.ack`는 **수신 측 commit 뒤에만** 보낸다. 전송 성공은 상대 저장의 증거가 아니다.
- 수신 처리 순서: 크기·파싱 → 구조·site·방향 검증 → 기존 처리 기록 확인 → 최초 처리분만 만료 판정과 업무 적용. 이미 처리한 메시지는 만료 때문에 결과가 바뀌지 않는다.
- 메시지 중복은 원본 프레임 text의 해시로, 업무 멱등성은 요청에만 `request_sha256`(예시 파일의 정규화 규칙)으로 판정한다. 같은 `command_id`에 다른 본문이면 `command.conflict`를 보내며 원래 command의 상태는 바꾸지 않는다.
- 재전송은 저장한 프레임 text를 그대로 보낸다(`message_id`·`sent_at` 유지). 저장은 `jsonb`가 아니라 text다.
- 응답 timeout은 결과 불명(UNKNOWN)이며 실패의 증거가 아니다. UNKNOWN은 같은 주문의 새 요청을 막고, 해제는 관리자의 명시적 대조로만 한다.
- 모든 조회·갱신은 인증된 세션의 site를 조건에 포함한다. 다른 site의 ID를 가리키는 ack·응답은 처리하지 않는다.

## 저장소

서비스 서버는 Flyway `V2`에 `wss_connections`, `message_outbox`, `message_inbox`, `wss_commands`를 추가한다(`V1`은 수정하지 않는다). 관제 서버는 버전 인식 migration runner를 만든 뒤 운영 PostgreSQL에 같은 구조를 둔다. 이 파일들은 아직 없다.

## 아직 정하지 않은 것

ack 재전송 주기, 명령 만료 길이, ping 주기·timeout, 재연결 백오프, 시계 허용 오차와 시계 이상 시 정책, inbox·outbox 보존 기간과 저장 한도, 같은 site 교체 절차, UNKNOWN 대조 주체·증거, 관제 상위 task 모델, `task.requested` 확장 필드는 실측과 합의 전까지 값을 정하지 않는다. 설정 키만 정의하고 WSS를 켤 때 필수로 검증한다. 예시의 `expires_at` 간격은 값이 아니라 형식을 보이기 위한 것이다.

## 검증과 수정

```bash
npx @asyncapi/cli@latest validate interfaces/wss/roport-wss.yaml
cd services/roport-control-server && uv run pytest tests/test_wss_contract_examples.py tests/test_layers.py
```

예시 값은 가짜다. 실제 토큰·호스트·IP를 넣지 않는다. 필드나 type을 바꾸면 yaml, examples.json, 이 README, 시험을 함께 고친다. [공개 저장소 보안 규칙](../../.agents/rules/public-repository-security.md)을 따른다.
