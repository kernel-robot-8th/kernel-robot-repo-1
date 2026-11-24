# 마커·외형 추종 서비스 연동 인수인계 (FFT1-207)

## 1. 목적과 적용 기준

고객이 마커 부착·휴대 가능 여부를 선택하면 `marker` 또는 `appearance` **한 가지 방식**으로 추종한다. 수령 확인·외형 촬영·대상 선택은 주행을 시작하지 않는다. 첫 출발은 관제 운영자의 명시적 승인 후 실행하고, 화면의 ‘추종 중’은 로봇이 실제 활성 상태를 보고한 뒤 표시한다.

| 항목 | 기준 |
| --- | --- |
| 통합 작업 분기 | `feature/inyup-baek/follow-mode-service` |
| 시작 기준 | `task/control-integration-test`의 `083276e5` |
| 작업 추적 | [FFT1-207](https://fp-team1.atlassian.net/browse/FFT1-207) |
| 이 문서의 범위 | 관제·서비스 서버·패드·MQTT 계약·가상 실행부와 실물 실행부의 연결 기준 |
| 실물 적용 상태 | **실제 ROS 실행부는 이번 작업에서 수정하지 않았으며, 두 모드의 서비스 경유 실물 추종은 아직 검증하지 않았다.** |

최종 PR 주소·커밋 SHA·통합 테스트 결과는 통합 담당자가 PR에 기록한다. 이전 실물 주행·Nav2·독립 추종 시험의 성공을 이 서비스 경유 통합 시험의 성공으로 대신하지 않는다.

## 2. 전체 동작 흐름

| 단계 | 서비스 패드·서버 | 관제 owner | Pinky 실행부 |
| --- | --- | --- | --- |
| 지원 방식 조회 | 현재 카트와 수령 인증에 연결된 지원 모드를 조회 | 현재 역할·기체 상태·현장 승인·`follow_modes`를 투영 | 실제 지원 여부를 확인한 실행 버전만 배포 |
| 수령·방식 선택 | `none` / `marker` / `appearance` 저장. marker만 카드 번호 35~49 사용 | `none`은 기존 복귀, marker는 `follow_pending`, appearance는 `follow_register` | 이 선택만으로 주행하지 않음 |
| 외형 촬영 | ‘내 모습 찾기’ / ‘다시 찍기’ 요청 | `follow_registration_command(operation=capture)` 발행 | 정지 상태에서 프레임·후보 생성. 주행 없이 비동기 완료 보고 |
| 사진 확인·선택 | 인증된 사진을 표시하고 후보 번호 또는 사각형 선택 | 현재 세션·프레임·revision 검증 후 `select` 발행 | 해당 프레임에서 대상을 등록하고 `target_ref` 반환. 아직 주행하지 않음 |
| 첫 출발 대기 | 등록 완료·관제 승인 대기 표시 | `follow_pending`. 운영자 `approve` 전에는 추종 명령 없음 | 정지 유지 |
| 첫 추종 시작 | 접수와 실제 추종 중을 구분해 표시 | 승인 후 `follow_target_command` 발행, 실제 활성 근거 확인 후 `following` | 선택된 모드만 활성화하고 구동 소유권 확보 후 `observe().follow.active=true` 보고 |
| 잠시 멈춤·재개 | `pause` / `resume` 요청 | 기존 명령 정지·실제 정차 확인 후 `follow_paused`. 유효한 세션·대상으로 명시적 재개 | 목표 갱신 중단·Nav2 취소·정지 보고. 재개 명령 전에는 움직이지 않음 |
| 복귀 요청 | ‘물품을 모두 내렸습니다 · 복귀 요청’ | 추종 중이면 먼저 정지 확인. 기존 N12 Nav2 복귀 → 라인 전환 → 주차 경로 사용 | 추종 구동권을 해제한 뒤 기존 복귀 실행 |
| 등록 해제·만료 | `release`, 만료 시 다시 촬영·선택 안내 | 오래된 참조를 출발 근거로 사용하지 않음 | 해제·재촬영·인식 재기동 시 이전 참조 무효화. 만료된 대상은 사용 금지 |

패드의 직접 MQTT·ROS 접근은 없다. 명령 경로는 **패드 HTTP → 서비스 DB/outbox → WSS → 관제 DB/단일 Fleet owner → MQTT → Pinky 실행부**다. 상태는 로봇 관측·결과가 같은 경로를 거쳐 돌아오며, HTTP 202·WSS ACK·MQTT PUBACK만으로 성공을 표시하지 않는다.

서비스 조작은 `follow.control_requested`의 고정 `request_id`, 수령 `receipt_id`, `robot_id`, `expected_revision`으로 중복·다른 카트·오래된 상태를 거른다. 관제 수신 후 실행 시 `sent_at`이 10초를 넘었거나 미래로 1초 넘게 어긋나면 거절 결과를 저장한다. 통신 복구 후 오래된 재개/복귀 요청을 뒤늦게 실행하지 않는다. 이 제한은 프레임/대상 TTL과 별개다.

## 3. 담당별 작업 경계

| 담당 | 맡을 구현·확인 | 공동 파일 충돌을 피하는 기준 |
| --- | --- | --- |
| 통합관제 담당 | 서비스·패드 선택 흐름, WSS/DB, Fleet 단계·명령·중복 방지, MQTT endpoint·schema, 가상 통합 테스트 | MQTT·서비스 규약 변경은 이 분기에서 관리. 로봇 ROS 내부를 대신 구현하지 않음 |
| 마커 추종 담당 | 기존 카드 35~49 식별·입력 신선도·목표 갱신을 유지하고, 실제 활성/정지 근거를 공통 드라이버에 제공 | 기존 `roport_follow`와 `trial_follow` 경로를 활용. appearance 등록 구현을 마커 코드에 섞지 않음 |
| 외형 추종 담당 | 정지 촬영·후보/ROI 선택·외형 등록, 세션/기동/만료 바인딩, 인증 사진 제공, 추종·유실·정지 근거 제공 | `roport_smartcart`의 탐지·특징·추적 코드를 재사용하되 독립 데모의 주행 owner를 그대로 함께 실행하지 않음 |
| Pinky 주 작업공간 통합 담당 | 두 모드 라우팅, 공통 `TrialDriver` 연결, 카메라·ROS·Nav2·guard 단일 소유권, 배포 묶음·기동 명령 | `trial_driver.py`와 launch/guard 변경 책임자를 한 명으로 정한다. 각 모드 담당자는 모듈/API를 제공하고 공동 라우팅 파일을 동시에 수정하지 않음 |

마커·외형 담당자가 동일인 또는 Pinky 통합 담당을 겸해도 경계는 유지한다. 실행부 PR은 이 계약 PR이 반영된 통합 기준에서 검토하고, 다른 팀원의 기능을 복사한 별도 시험 사본을 최종 기준으로 삼지 않는다. 통합 담당자가 실제 PR base·의존 PR·검증 SHA를 맞춘 뒤 병합 순서를 정한다.

기존 구현 참고:

- 마커 입력: [`roport_follow/ros_follow.py`](../../roport-pinky/src/roport_follow/roport_follow/ros_follow.py), [`trial_follow.py`](../../roport-pinky/src/roport_pinky/roport_pinky/trial_follow.py).
- 외형 인식/독립 실행: [`shopping_follow.py`](../../roport-pinky/src/roport_smartcart/roport_smartcart/shopping_follow.py), [`appearance_reid.py`](../../roport-pinky/src/roport_smartcart/roport_smartcart/appearance_reid.py).
- 공통 MQTT 경계: [`field_pinky.py`](../src/roport_control_server/adapters/mqtt/field_pinky.py), [`field_wire.py`](../src/roport_control_server/adapters/mqtt/field_wire.py).

## 4. 실제 실행부의 공통 연결 계약

새로운 MQTT 수신기나 별도 주행 스레드를 추가해 제어권을 나누지 않는다. 기존 endpoint가 명령을 검증하고, 실행부는 다음 연결점을 구현한다.

| 연결점 | 필수 동작 |
| --- | --- |
| `validate_action(command)` | 읽기 전용 사전 검사. 지원 모드·지도/세션/대상·정지/준비 상태를 검사하며, 여기서 촬영·등록·주행을 시작하지 않음. 미지원 등록/모드는 `ValueError("unsupported_operation")` |
| `dispatch(command)` | 검증된 작업을 기존 비동기 실행 경로에 접수. `follow_registration.requested`는 촬영·선택·해제만 수행하며 모터 목표를 발행하지 않음 |
| `drain_results()` | `(원래 command, "completed" 또는 "failed", details)` 반환. 요청 correlation을 그대로 보존. 사진/특징 벡터 대신 아래의 결과 근거만 반환 |
| `observe()` | 기존 상태에 선택 필드 `follow` 추가. 실제 선택 모드·대상·구동 활성 여부를 보고. 접수 직후 추정값을 활성 근거로 쓰지 않음 |
| `safe_stop(reason)` | 현재 추종 목표 갱신·Nav2 실행을 중단하고 독립 guard를 통해 정지. 정지 응답은 실제 정차 관측으로 확인 |

endpoint는 외부 `follow_registration_command`를 내부 `follow_registration.requested`로, `follow_target_command`를 `follow.requested`로 변환한다. `expected_boot_version`은 내부 `expected_boot_id`로 변환하고 배포 설정의 `binding_id`, `map_id`를 붙인다. 이 내부 표현은 새로운 MQTT 토픽/규약이 아니다.

등록 완료 시 현재 관측이 정지·라인 비활성·Nav2/위치 추정 준비·신선한 상태여야 한다. 잘못된 등록 결과는 성공으로 넘기지 않고 `invalid_registration_result`로 정지·실패 처리한다. 실행 오류를 driver 결과 큐로 보고해야 하며 runner를 예외로 종료하는 방식으로 처리하지 않는다.

`follow` 관측 예시는 다음과 같다. appearance에서는 아래 세 참조가 모두 현재 시작 명령과 일치해야 한다.

```json
{
  "active": true,
  "follow_mode": "appearance",
  "follow_session_id": "follow-session-demo-1",
  "target_ref": "target-demo-1",
  "perception_boot_id": "perception-demo-1"
}
```

마커의 기존 요청은 session 필드가 없을 수 있으므로 `{ "active": true, "follow_mode": "marker" }`도 지원한다. 요청에 session이 있으면 같은 값을 보고한다. 마커 실행부도 이 관측이 없으면 MQTT 접수는 가능하지만 서비스는 ‘시작 확인 대기’를 유지한다.

endpoint는 실제 모드·대상·Nav2 준비·구동 소유권·관측 신선도가 확인된 경우에만 `in_progress.result.follow_active=true`와 `result.follow`를 발행한다. Fleet도 이 활성 값, 중첩된 `result.follow`의 mode/session/target/perception, 현재 상태 근거가 시작 명령과 일치하는지 다시 검사한다. 고객이 서 있으면 추종이 활성인 상태에서도 속도가 0일 수 있으므로 속도 0/이동 여부 자체를 활성 판정으로 사용하지 않는다. `accepted`는 활성 근거가 아니며, `drain_results()`에서 임의로 `follow_active=true`를 넣어도 이를 근거로 인정하지 않는다.

## 5. MQTT 필드와 등록 결과

토픽은 기존 `roport/robot/{pinky_id}/cmd`, `/event`, `/ack`를 유지한다. QoS 1, retain=false, 현재 기동 확인, 명령 만료, journal/outbox, 결과 저장 ACK, watchdog 및 단일 활성 명령 규칙도 유지한다. 개인정보·주문 상세·영상 원문·특징 벡터를 MQTT에 싣지 않는다.

### 5.1 추종 시작

| 방식 | `follow_target_command.payload` |
| --- | --- |
| 기존 마커 호환 | `expected_boot_version`, `map_version`, `card_id`(35~49). `follow_mode` 생략 시 marker |
| 명시적 마커 | 위 필드 + `follow_mode:"marker"`, 선택적 `follow_session_id` |
| 외형 | `expected_boot_version`, `map_version`, `follow_mode:"appearance"`, `follow_session_id`, `target_ref`, `perception_boot_id`. `card_id` 금지 |

`max_duration_s`는 두 방식에서 사용할 수 있는 선택 필드이며 1~240초다. 관제는 자신의 명령 제한 시간보다 짧게 설정한다. 외형 실행부는 이 시간과 별개로 대상 등록 만료를 지켜야 한다. 모드가 준비되지 않았다고 다른 방식으로 자동 전환하지 않는다.

### 5.2 외형 등록

`follow_registration_command` 공통 필드는 `expected_boot_version`, `map_version`, `follow_session_id`, `operation`이다.

| operation | 추가 요청 | 완료 details |
| --- | --- | --- |
| `capture` | 없음 | `follow_session_id`, `frame_ref`, `image_ref`, `expires_at`, `perception_boot_id`, `candidates:[{candidate_id,box}]` |
| `select` | `frame_ref` + `candidate_id` **또는** `box` 하나 | `follow_session_id`, `target_ref`, `expires_at`, `perception_boot_id` |
| `release` | 없음 | `follow_session_id`, `cleared:true` |

MQTT `completed.result`에는 endpoint의 관측 상태가 함께 붙고 `stopped:true`가 필수다. `box`는 영상 전체 기준 `[x1,y1,x2,y2]`, 각 값 0~1, `x1<x2`, `y1<y2`다. 후보 ID는 한 프레임에서 중복되면 안 된다. `frame_ref`와 후보 ID는 실제 같은 촬영 결과에 속해야 하며, 화면에서 선택한 ROI를 현재 다른 영상에 적용하면 안 된다.

`perception_boot_id`는 **인식 프로세스의 기동 세대**다. MQTT endpoint의 `boot_version`과 별개이며 인식 재기동 때 새 값으로 바꾼다. 프레임/대상 참조는 세션·기체/바인딩·지도·endpoint 기동·인식 기동·유효 기간에 연결한다. 다른 고객·다른 기체·재촬영 전 프레임·이전 인식 세대·만료 대상 참조는 거절한다. 해제는 해당 세션의 사진·특징·대상 상태를 제거하며 다른 세션을 해제하지 않는다.

`expires_at`은 시간대가 포함된 미래 ISO 시각이다. 실제 실행부의 TTL과 삭제 정책은 담당자가 제시해 현장 합의하고, 관제는 받은 만료를 그대로 검사한다. 가상 구현은 촬영부터 120초이며 만료 중 추종도 정지한다. 이 120초를 검증 없이 실물의 최종 운영 정책으로 확정하지 않는다.

전체 envelope 예시와 닫힌 schema는 [MQTT README](../interfaces/mqtt/README.md), [examples.json](../interfaces/mqtt/examples.json), [AsyncAPI](../interfaces/mqtt/roport-mqtt.yaml)를 따른다. `follow_session_id`와 `target_ref`는 추종 세션/대상의 불투명 참조이며 얼굴 식별자나 원본 이미지가 아니다.

## 6. 사진 전달은 별도 인증 HTTP

사진 경로는 **Pinky/인식 서버 제공자 → 관제 프록시 → 서비스 프록시 → 인증된 카트 패드**다. 패드가 로봇 URL이나 토큰을 직접 받지 않는다.

| 경계 | 계약/설정 |
| --- | --- |
| 실제 사진 제공자 | `GET /frames/{image_ref}`, `Authorization: Bearer <배포 시 주입한 토큰>`. 현재 세션/프레임·인식 기동·만료를 검사하고 JPEG 또는 PNG 반환 |
| 관제 → 제공자 | `ROPORT_FOLLOW_FRAME_SOURCES`의 기체별 origin과 `ROPORT_FOLLOW_FRAME_TOKEN` 사용 |
| 서비스 → 관제 | 관제 `/api/fleet/follow/{session_id}/frames/{frame_ref}`. 서비스 `SERVICE_FOLLOW_PREVIEW_BASE_URL`, `SERVICE_FOLLOW_PREVIEW_TOKEN`과 관제 `ROPORT_FOLLOW_PREVIEW_TOKEN`을 맞춤 |
| 패드 → 서비스 | 기존 카트 인증으로 `/cart-api/follow/{receiptId}/frames/{frameRef}` 사용. 현재 카트·수령 세션에 속한 프레임만 제공 |

현장 PC 사이에는 HTTPS와 인증서 검증을 사용한다. HTTP 예외는 루프백 가상 시험만 허용한다. 실제 토큰·인증서/키·프레임 원문은 Git에 넣지 않고 배포 시 전달한다. 관제는 배포 설정의 origin만 호출하며 MQTT/고객 입력 URL을 받아 요청하지 않는다. 리다이렉트, 허용되지 않은 이미지 형식, 2 MiB 초과 응답, 만료/교체된 프레임은 거절한다. 응답은 `Cache-Control: no-store`다.

실제 사진 제공자 및 TLS 배포는 아직 Pinky/인식 담당의 연동 작업이다. 이번 가상 제공자는 [`virtual_follow_frames.py`](../scripts/virtual_follow_frames.py)이며 `--preview-port`로 명시했을 때만 루프백에서 합성 JPEG를 제공한다. 그림에는 `VIRTUAL FOLLOW TEST`가 표시되고 실제 카메라를 사용하지 않는다.

## 7. 카메라·ROS·구동권 충돌 방지

1. **카메라 물리 장치 소유자는 하나로 정한다.** 기존 마커용 Picamera 스트림과 외형용 카메라 드라이버가 같은 장치를 각각 열지 않는다. 한 캡처 제공자의 영상을 재사용하거나, 정지 상태에서 기존 프로세스를 완전히 해제한 뒤 새 소유자를 시작한다. 영상 방향·해상도·왜곡 보정과 ROI 좌표 기준을 함께 합의한다.
2. **구동 owner는 하나다.** 관제 runner가 기존 guard/lease를 소유하고, 선택된 추종 방식만 Nav2 목표를 갱신한다. 독립 Shopping Nav2 client, `doll_follow_demo`의 `line_nav/cmd_vel`/lease 발행, 기존 TrialDriver를 동시에 주행 owner로 실행하지 않는다. 외형 탐지 재사용과 독립 데모 구동 재사용은 구분한다.
3. **ROS 범위를 기체별로 고정한다.** 동일 기체의 영상·TF·odom·Nav2·추종 노드가 같은 합의된 domain/namespace를 사용해야 한다. 여러 기체의 동일 이름 토픽이 섞이지 않게 한다. 마커의 기존 `/follow/target`, `/follow/status`와 외형 상태를 무조건 같은 토픽으로 합치지 말고 모드 선택 adapter가 입력을 구분한다.
4. **전환 전에 정지와 해제를 확인한다.** 이전 목표 취소·실제 정차·이전 모드 출력 중단을 확인한 뒤 새 모드/복귀를 허용한다. 대상 유실·애매한 일치·카메라 단절·기동 변경·guard 오류는 정지/거절 근거다. marker↔appearance 자동 전환이나 임의의 다른 사람 재선택은 하지 않는다.
5. **최대 주행 속도는 0.08 m/s 이하로 맞춘다.** Nav2 추종·외형 실행부의 실제 속도 설정도 담당자가 검증한다. 이번 MQTT 추종 명령에 새로운 속도 필드를 추가한 것은 아니므로 관제 라인 속도 설정만 보고 추종 속도도 제한됐다고 판단하지 않는다. 기존 N12 정밀 접근의 0.02 m/s와 허용 오차·guard 조건은 유지한다.

## 8. 현장 허용 설정과 적용 순서

`follow_modes`는 실행부의 지원 목록을 새 MQTT 광고로 추정하는 기능이 아니다. **담당자가 연동을 검증한 뒤 배포 설정에 등록하는 허용 목록**이다. hardware에서는 `field_verified:true`, 현재 smartcart 역할, `verified_follow_robot` 일치도 필요하다. 확인하지 않은 기체에 승인값을 복사하지 않는다.

설정 예시는 다음과 같다. 실제 확인 전에는 `field_verified:false`, `verified_follow_robot:null`, 해당 기체의 허용 목록을 빈 배열로 유지한다.

```json
{
  "mode": "hardware",
  "field_verified": false,
  "verified_follow_robot": null,
  "follow_modes": {"pinky3": []}
}
```

`follow_modes`에서 기체 항목을 생략하면 기존 호환을 위해 marker 기본값이 적용될 수 있다. 사용 금지는 명시적인 `[]`로 표현한다. 현장 확인 후 통합 담당자가 실제 기체 ID와 검증된 모드만 넣는다. 서비스 패드에서는 `VITE_PAD_FOLLOW=1`과 서버 연결이 필요하며, 현재 상태 조회에서 제공된 지원 모드만 선택할 수 있다.

적용 순서:

1. 통합 담당자와 모드 담당자가 이 계약·기동 방식·카메라 소유자·공동 `TrialDriver` 수정 책임을 먼저 맞춘다. 실물 런처/파라미터·모델/보정 파일의 별도 배포 항목을 기록한다.
2. 각 모드 코드를 공통 드라이버에 연결하고 ROS 없는 단위 테스트로 거절·세션/기동/만료·정지·결과 근거를 검증한다. 기존 마커 요청과 라인/Nav2 회귀도 확인한다.
3. 관제·서비스·패드·가상 Pinky 전체 흐름을 검증한다. [가상 전체 연결 검사](follow-service-e2e.md)를 사용하고, 운영 DB·현장 브로커와 분리한다.
4. 실물은 처음에 **주행 없이** 기동·지도·카메라/TF·정지 관측·촬영/선택·인증 사진·등록 해제를 확인한다. unsupported/만료/다른 세션에서 명령이 거절되는지 확인한다.
5. 배치·통로·정지 장치 확인 후 한 모드씩 명시 승인해 출발·정지·유실·재개·복귀를 확인한다. 실물 통과 SHA와 설정을 기록한 뒤 그 기체의 허용 모드를 확장한다.

## 9. 통합 완료 판단과 남은 범위

이번 작업에서 관제/서비스/패드/MQTT와 가상 실행부를 연결한다. 실제 `roport_smartcart`의 HTTP/ROS 실행기를 서비스 owner에 연결한 것으로 보지 않는다. 기존 패드 [`Shopping.tsx`](../../roport-web/apps/pad/src/Shopping.tsx)의 독립 Shopping 데모도 이 통합 완료에 포함하지 않는다. 서비스 수령 뒤 `FollowSession` 경로를 사용하며 별도 데모의 ‘시작’ 버튼으로 운영 권한을 우회하면 안 된다.

실물 통합 PR의 완료 조건:

| 검사 묶음 | 통과 근거 |
| --- | --- |
| 호환·구동권 | 기존 marker payload 정상, appearance는 등록 없으면 거절, 한 mode/owner만 출력, 새 노드 중복 기동 거절 |
| 주행 전 비주행 | 수령·상태 GET·촬영·선택·사진 조회 중 이동 없음, 관제 첫 승인 전 `follow_target_command` 없음 |
| 등록과 개인정보 | 틀린 기체/세션/프레임/인식 기동/만료 거절, 재촬영·해제 후 이전 프레임과 대상 사용 불가, 인증 없는 사진 조회 거절, 원문/특징은 MQTT·로그/Git에 없음 |
| 실제 상태·정지 | accepted만으로 following 표시 없음, 실제 관측 후 활성화, pause/유실/통신 단절 시 정지, 정차 확인 전 재개/복귀 없음, 위험 시 다른 모드 자동 전환 없음 |
| 기존 서비스 회귀 | 선반 출고·수령·N12 Nav2 복귀·라인 전환 유지, 최고속도 ≤0.08 m/s, 기존 0.02 m/s 정밀 접근 보존, 프로세스 재기동 뒤 무단 재출발 없음 |

가상 테스트는 계약과 상태 전이를 검증한다. 실제 카메라 품질·대상 재식별·장애물 회피·정지 거리·서로 다른 PC의 시간 동기화·ROS 통신·인증서 배포는 별도 현장 검증 대상이다. 최종 테스트 수와 실물 결과는 이 문서에 추정값을 넣지 않고 PR 검증 기록에서 확인한다.

## 10. 팀원에게 바로 전달할 내용

> 고객이 마커 가능을 선택하면 기존 카드 35~49 추종, 불가능을 선택하면 외형 등록 추종으로 연결합니다. 동시에 구동하지 않고 첫 출발은 관제 승인 후에만 합니다.
>
> 마커 담당은 기존 입력/주행을 유지하면서 실제 `observe().follow` 활성·정지 근거를 공통 TrialDriver에 연결해 주세요. 외형 담당은 `follow_registration.requested`의 capture/select/release와 세션·기동·만료에 묶인 target_ref, 인증된 `/frames/{image_ref}` 제공자를 연결해 주세요. 두 방식 모두 기존 `validate_action → dispatch → drain_results`와 guard/lease를 사용합니다.
>
> 실제 ROS 연결은 아직 이번 PR 범위에 들어 있지 않습니다. 독립 Shopping/Nav2/속도 발행기를 관제 runner와 함께 띄우지 말고, 카메라/ROS/구동 소유자를 먼저 합의해야 합니다. 최고속도 0.08 m/s 이하와 N12 정밀 접근 0.02 m/s는 유지합니다. 유실 시 자동으로 다른 모드로 바꾸지 않습니다.
>
> 확인해서 알려주실 값은 ① 실행 진입점·필요 launch/환경과 카메라 소유 방식 ② 각 모드의 실제 활성/정지 보고 연결점 ③ 외형 사진 제공자 origin·TLS 구성·TTL/인식 기동 처리 ④ 속도/guard 설정 ⑤ 적용 브랜치·SHA와 시험 근거입니다. 실제 비밀값·인증서 키·고객 사진은 Git/채팅에 넣지 말고 배포 경로로 전달해 주세요.
>
> 계약 기준은 `feature/inyup-baek/follow-mode-service` / FFT1-207이며, 실제 PR의 기준 커밋과 병합 순서는 통합 담당자가 맞추겠습니다.
