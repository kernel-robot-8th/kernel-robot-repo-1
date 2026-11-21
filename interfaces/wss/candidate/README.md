# 관제 WSS 로컬 구현 후보 — 합의 전

현재 로컬에서 개발한 예약·수령 흐름을 서비스 담당자가 비교 검토하기 위한 계약이다.
**상위 폴더의 S1 제안을 대체하거나 구현 완료했다고 선언하는 문서가 아니다.**
두 규약은 호환되지 않는다. Fleet 실행부·서비스 Java·고객 웹을 명시적 로컬 시험 설정에서 연결한다.
운영 배포는 양측 계약 합의와 현장 검증 후 진행한다.

- 공용 입력/출력: [examples.json](examples.json). Python 저장소·WebSocket 시험과 Java 계약 시험이 이 파일을 읽는다.
- 관제 구현/검증 방법: [관제 README](../../../services/roport-control-server/README.md#서비스-wss-관제-기반-합의-전-후보).
- 담당 협의: FFT1-183. 관제 분리 구현: FFT1-164.

## S1과 비교하여 결정할 것

| 항목 | 기존 S1 제안 | 이번 로컬 구현 후보 |
| --- | --- | --- |
| 첫 업무 | 요청을 항상 거부 | 작업 저장·접수·진행·완료·실패·수령 기록 |
| 송수신 주체 | `service` / `control` | `service-01` / `control-01` |
| 업무 중복 기준 | `command_id` + `request_sha256` | 예약의 `reservation_id`, 명세의 `order_revision` |
| 요청 필드 | `pickup_id`, `pickup_location_id`, `package_revision` | `reservation_id`, `pickup_point_id`, `pickup_at`, `operation`, 선택 `marker` |
| 전달 확인 | `status=STORED/REJECTED`와 오류 코드 | ACK는 저장 성공만 확인. 오류는 ACK하지 않음 |
| 재전송 | 저장한 프레임 text 그대로 | JSONB 객체를 재직렬화. ID·시각·내용 유지, 공백/키 순서는 보장하지 않음 |
| 만료·복구 | `expires_at`, UNKNOWN, cursor/snapshot | 아직 없음. timeout을 실패로 바꾸지 않고 미확인 이벤트 재전송 |
| 연결 소유권 | 새 연결이 기존 연결을 4001로 교체 | 서비스는 기존 연결을 4001로 교체, 관제는 재접속 종료 |
| 서버 인증 저장 | 토큰 SHA-256 해시 | 서비스는 환경 주입 토큰을 상수 시간 비교. 운영 해시 저장은 합의 필요 |
| DB 배포 | 버전 migration runner | 명시적 반복 설치 SQL. 운영 버전 migration은 미포함 |

두 계약의 `schema_version`이 모두 `1.0`이어도 같은 계약이라는 뜻이 아니다.
합의 시 하나로 맞춘 예시와 검증 코드를 양쪽에 적용한다. 자동 감지/혼용하지 않는다.
서비스 Java와 Python이 같은 예시를 검증하며 각 DB 저장 뒤 ACK한다.

## 연결과 처리

관제 → `wss://example.com/ws/v1/sites/test/control`, HTTP Upgrade의 `Authorization: Bearer`.
평문 `ws`는 loopback만 허용하며 예제·테스트용이다. 원격 연결은 기본 인증서 검증을 쓰는 WSS만 허용한다.
URL의 사용자정보/query/fragment를 금지하고, 경로의 site와 설정값을 대조한다.
수신 최대 크기는 64 KiB다. 현재는 관제 DB 하나를 현장 하나에만 사용한다.
다른 현장의 outbox를 전송하지 않고, ACK도 해당 현장의 메시지만 갱신한다.

1. 요청의 inbox + 작업 + 접수 outbox를 같은 트랜잭션으로 저장한다.
2. commit 후 ACK를 보낸다. ACK 유실로 같은 요청이 와도 작업·접수 이벤트는 늘어나지 않는다.
3. 작업 상태 + 결과 outbox도 함께 저장한다. task 행 잠금 뒤 할당한 seq 순서로 최대 100개씩, 상대 ACK 전까지 1초 주기로 재전송한다.
4. 네트워크 단절·서비스 저장소 DB 오류는 ACK 없이 1초 후 재접속한다. 4001은 다른 연결로 교체된 것으로 보고 자동 재접속을 끝낸다.

잘못된 형식은 저장·ACK하지 않는다. ID 충돌/지원하지 않는 업무 변경은 정상 작업으로 적용하지 않는다.
형식 오류는 ACK 없이 건너뛰고 다음 프레임을 처리한다. 유효한 envelope의 업무 거절은 site/message_id/내용 해시/오류 코드로 격리 저장한 뒤 ACK하므로 뒤의 요청·수령·ACK를 막지 않는다. 같은 거절의 재전송은 최초 판정을 유지한다. 업무 승인은 ACK가 아니라 task.accepted/진행으로 확인하며, 접수 없는 ACK는 서비스 담당/운영자가 관제 격리 기록을 확인한다.
오류 응답·재시도 상한·불명 상태 대조 정책은 합의 후 구현해야 한다. `task.rejected`는 저장된 작업을 업무적으로 거부하는 결과이며 파싱 오류의 응답이 아니다.

## 최종 JSON의 공통 필드

각 WebSocket text frame은 JSON 객체 하나다. 아래 필드를 공통으로 가진다.
JSON 예시는 공용 파일에 모든 필드를 펼쳐 두었다.

| 필드 | 의미 / 예시 |
| --- | --- |
| `schema_version` | 후보 형식 표기 `"1.0"`; S1과 호환성을 보장하지 않음 |
| `message_id` | 전송 메시지 고유 ID. 재전송은 동일 ID·내용·시각을 유지 |
| `type` | 아래 8종 중 하나 |
| `site_id` | 접속 경로의 현장 키, 예: `test`. 영문/숫자/`_`/`-`, 1–128자 |
| `source`, `target` | 서비스→관제는 `service-01`, `control-01`; 응답은 반대 |
| `sent_at` | timezone 포함 ISO 시각, 예: `2030-01-01T00:00:00Z` |
| `task_id` | 생성한 관제 작업 ID. 상태 이벤트와 수령 메시지에만 존재 |
| `payload` | type별 본문. 아래 정의 참조 |

이 후보의 수신 envelope와 payload는 알려진 필드만 허용한다. 알 수 없는 필드를 조용히 버리지 않는다.
요청과 ACK에는 `task_id`를 넣지 않는다. ACK 자체에는 ACK를 보내지 않는다.

## 메시지 8종

| type | 방향 | 의미 |
| --- | --- | --- |
| `task.requested` | 서비스 → 관제 | 예약·포장 목록 접수. 같은 예약의 새로운 명세 revision도 이 type 사용 |
| `task.accepted` | 관제 → 서비스 | 작업이 저장됨. 출발·완료를 의미하지 않음 |
| `task.progress` | 관제 → 서비스 | 작업 상태/진행 변경, 명세 갱신 수락·거부 결과 |
| `task.completed` | 관제 → 서비스 | 작업 전체 완료. 고객 수령 사실과 별개 |
| `task.failed` | 관제 → 서비스 | 저장된 작업의 실패. 통신 timeout으로 생성하지 않음 |
| `task.rejected` | 관제 → 서비스 | 저장된 작업의 업무 거절 |
| `handover.receipt_recorded` | 서비스 → 관제 | 서비스가 명시적으로 저장한 포장품별 고객 수령 기록 |
| `message.ack` | 양방향 | `payload.message_id` 메시지의 저장 완료 |

### 요청 본문

| 필드 | 의미 / 검증 |
| --- | --- |
| `order_id` | 주문 ID. 현장 DB에서 주문당 활성 작업 하나 |
| `reservation_id` | 예약 ID. 같은 예약은 기존 작업 반환 |
| `operation` | `prepare_order` 입고 준비 / `customer_service` 고객 출고 요청 |
| `packages` | 포장품 목록. 빈 목록은 `WAITING_MANIFEST`, 아직 배차 불가 |
| `packages[].package_id` | 업무 포장품 ID. 목록에서 중복 불가 |
| `packages[].quantity` | 물리 포장품 하나이므로 정수 `1` |
| `packages[].marker` | 선택 물리 마커 문자열. `15,16,17,19,20,21,22` 중 하나, 목록에서 중복 불가. `18`은 TRAY 예약 |
| `pickup_point_id` | 목적지 키, 예: `G1`. Fleet의 보정된 목표점 키와 대조 |
| `pickup_at` | 고객 도착 희망 시각. timezone 필요. 현재 저장소는 만료/배차 시각을 판단하지 않음 |
| `order_revision` | 서비스가 증가시키는 정수, 0 이상 |

새 `message_id`로 같은 예약을 보내도 별도 작업을 만들지 않는다.
포장 목록만 높은 revision으로 변경 가능하며, 배차 claim 뒤에는 거부 결과를 저장·회신한다.
목적지·예약 시각·operation 변경은 이 경로에서 지원하지 않는다.
`claim(task_id, order_revision)`은 자식 로봇 작업 생성 전에 DB에 명세 잠금을 기록한다.
Fleet service_tick은 claim 뒤 포장품별 입출고 작업을 생성한다.

### 상태 이벤트 본문 (accepted/progress/completed/failed/rejected 공통)

| 필드 | 의미 |
| --- | --- |
| `order_id`, `reservation_id` | 원 요청의 주문·예약 |
| `task_revision` | 관제가 증가시키는 진행 revision. 최초 접수 0, 변경마다 +1 |
| `state` | `QUEUED`, `WAITING_MANIFEST`, `RUNNING`, `HOLD`, `COMPLETED`, `FAILED`, `REJECTED` |
| `progress` | 진행 객체. Fleet은 stages 목록과 packages[{package_id,robot,stage}]로 진행을 전달 |
| `progress.manifest_change` | 갱신한 `order_revision`, 적용 여부 `accepted`, 미적용 `reason` |

명세 미적용 이유는 `order_revision_not_newer`, `dispatch_already_locked`다.
예약 기본 필드 변경은 저장하지 않고 오류로 처리한다.
종료 상태에서 RUNNING으로 되돌릴 수 없다. 같은 상태·진행 재저장은 이벤트를 늘리지 않는다.
서비스는 높은 task_revision만 반영하며 종료 상태를 RUNNING으로 되돌리지 않는다.

### 수령 본문

`receipt_id`(수령 고유 ID), `order_id`, `reservation_id`, `package_ids`(비어 있지 않은 포장품 ID 목록).
메시지의 `task_id`·주문·예약·현장이 기존 작업과 일치하고 모든 포장품이 그 작업의 명세에 있어야 한다.
같은 message_id의 다른 내용과 같은 receipt_id의 다른 포장품 목록을 거부한다.

최초 저장 시 관제 스냅샷에서 각 포장품의 `stage=await_receipt`, `status=AWAITING_RECEIPT`를 확인해
DB 내부 `arrival_verified`를 기록한다. 조기 수령은 false로 보존하며 나중 도착/재전송으로 true가 되지 않는다.
ACK는 기록 저장이며 출발 허가가 아니다. 서비스는 조기 수령의 ACK를 성공으로 취급하지 말고 운영자에게 상태 대조를 요청해야 한다. 도착 후 새 수령 의사 확인과 새 receipt가 필요하다. 자동 재확인/복구 신호는 이번 후보에 없다.
정상 수령의 재전송은 복귀 뒤에도 기존 판정을 유지한다. 기존 근거 없는 이력도 false로 남긴다.

## 병합 전 합의 / 이후 작업

- S1 또는 후보 중 사용할 계약과 식별자·오류/만료·복구 정책을 정하고 두 서버 예시를 일치시킨다.
- 서비스 인증 저장·연결 교체·운영 migration·외부 TLS/프록시 조건을 검증한다.
- 서비스 Java → Fleet → MQTT 가상 로봇 → 고객 수령을 통합 시험한다. 자동 시작·실예약·실물 구동은 포함하지 않는다.
