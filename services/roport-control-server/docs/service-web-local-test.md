# 고객 웹 → 서비스 → 관제 가상 연동 시험

## 실행

저장소 `services/roport-control-server`에서 실행한다.

```bash
./tools/run-latest-service-test.sh
```

Java 21, Node 24, uv, PostgreSQL 16, Mosquitto가 필요하다. 기존 시험 런타임 캐시가 있으면 자동 사용하며 다른 PC에서는 `PG_BIN`, `MOSQUITTO_BIN`을 지정한다. 포트가 사용 중이면 `--port 8001`을 붙인다. 매번 현재 소스에서 Java를 빌드하고 Python/Vite를 실행한다. Git pull은 자동으로 수행하지 않는다.

1. 출력된 **관제 주소**에 접속해 조작 키를 입력하고 **요청 처리 시작**을 누른다. 실행기는 소형·대형 시험 입고 2건을 대기열에만 넣으며, 버튼을 누르기 전에는 로봇이 출발하지 않는다. 키 입력·화면 새로고침만으로도 시작되지 않는다. 선반 보관 완료를 확인한 뒤 다음 단계로 간다. 선반 재고나 로봇 완료 상태를 DB에 직접 넣지 않는다.
2. **고객 웹 주소**에 접속한다. `e2e-login`은 소형 물품, `large-login`은 대형 물품이다. 수령 위치·항공편·시간을 선택해 예약한다.
3. **진행 상태**에서 카트의 이동·도착을 확인한다. 시험 환경에서만 예약 시각 대기 없이 출고한다. 운영 기본 출고 정책(예약 5분 전)은 바꾸지 않는다.
4. 도착 후 **카트 패드 주소**에 예약번호를 입력하고 수령 확인을 누른다. 서버 수령 기록 → WSS → 관제 도착 증거 확인 → 복귀 순서로 진행한다. 도착 전 또는 다른 카트에서는 수령이 거부된다.
5. Ctrl+C로 종료한다. 다시 같은 명령을 실행하면 새 시험 환경이 만들어진다. 이전 시험 기록을 덮어쓰지 않는다.

처음 실행은 빌드 시간이 추가되고, 가상 주행은 기존 0.08m/s 속도를 유지한다. 소형·대형 연속 시험에는 수 분이 걸린다. 관제 UI의 조작 키는 실행 출력 마지막 줄에 표시한다.

## 연결과 저장

```text
고객 React 웹 → Java 서비스 API → 서비스 PostgreSQL
                                  ↕ WebSocket (로컬 ws)
카트 패드 → 수령 API          FastAPI / Fleet 실행부 → 관제 PostgreSQL
                                  ↕ MQTT
                           가상 Pinky 3대 / OMX
```

- 모든 서버는 loopback에만 열린다. 매 실행 새 `service_test`·`control_test` DB를 사용하며 운영 DB에는 연결하지 않는다.
- 기록: `~/roport-trial/service/<실행시각>/`. `run.json`에 주소와 실행 설정, `database/`에 종료 후 DB 파일, `fleet/`에 로봇 기록과 로그가 남는다. 관제 실행 상태는 이 시험의 관제 DB에 저장한다.
- `service-env.json`·`ui-key.txt`는 실행용 임시 인증 정보를 포함하므로 Git에 올리지 않는다. 기본 디렉터리 권한은 700, 파일은 600이다.
- `GET /api/pickups/{pickupId}/progress`는 로그인한 고객의 예약만 조회한다. `state`, `stages`, `updatedAt`, `receivedPackages`, `orderComplete`를 반환한다. `orderComplete`는 관제 완료 이벤트가 아닌 서비스 수령 기록에서 나온다.
- 패드는 확인 응답의 `verificationId`를 보존하고 `POST /cart-api/receipts`에 보낸다. 같은 확인의 재전송은 같은 `confirmationRequestId`를 사용한다.

## 검증

실행기가 켜진 상태에서 웹 디렉터리에서 실행한다. 새 시험 실행의 `run.json` 경로를 지정한다.

```bash
ROPORT_SERVICE_RUN="$HOME/roport-trial/service/<실행시각>/run.json" \
  npm exec -- playwright test --config service-fleet.config.ts
```

이 검사는 시작 승인 전 대기를 확인하고 관제 화면의 시작 버튼을 누른 뒤, 실제 브라우저 예약·패드 입력으로 소형/대형 출고·수령·복귀를 확인한다. 중복 예약, 도착 전 수령, 다른 카트 수령, 중복 수령도 검사한다. 기존 `tests/service_fleet_e2e.py`는 실제 MQTT/로봇 드라이버의 Nav2 명령 입력 경계 검증을 유지한다.

## 구현 경계

- 포장 담당자의 운영 API가 아직 없어 포장 확정 입력은 로컬 시험 시드로 공급한다. 선반 입고부터는 실제 관제 판단과 MQTT 완료 이벤트를 사용한다. 운영 포장 UI/API를 구현한 것으로 취급하지 않는다.
- 서비스 WSS는 로컬에 구현된 outbox/inbox 규약을 사용한다. `interfaces/wss/README.md`의 S1은 아직 합의 전 제안이며 이번 작업에서 확정 규약으로 바꾸지 않았다.
- 기존 고객 QR 렌더링·재발급 및 서버 검증을 유지한다. 카메라 인식은 이번 범위에 없다. 브라우저 시험은 예약번호를 사용한다. 안내·추종 기능은 서버 모드에서 표시하지 않는다.
- OMX 실제 모터·카메라, 실물 Nav2 결과, 실제 인증/외부 HTTPS 배포는 별도 현장 검증이 필요하다. 물리 I/O만 가상이며 임의 완료로 관제 단계를 건너뛰지 않는다.

### 서비스 요청을 두 대의 현장 Fleet에 연결하기

`tools/prepare_field.py --output <새 폴더> --two-cart-trial`로 두 역할의 번들을 생성한다. 이 시험 배정은 물류 Pinky2(N17), 스마트카트 Pinky1(H1)이다. 현장에서 지도·출발 방향·N12 복귀 목표·선반 모델 및 보정 버전을 확인하고 번들의 현장 검증 절차를 마친다.

관제 PC에서 기존 시험용 브로커와 외부 Pinky 두 대, 선반 Host를 준비한 뒤 아래처럼 실행한다. 이 모드에서는 실행기가 창고 OMX3를 모의하므로 같은 브로커의 실제 OMX3 Host는 종료해 둔다.

```bash
uv run python tools/run_service_fleet.py --field-bundle "$FIELD_BUNDLE" --broker-port "$BROKER_PORT" --hardware-authorized --shelf-clear-confirmed --real-shelf 2b
```

위 명령에서 슬롯을 생략하고 `--real-shelf`만 지정해도 `2b`를 사용한다. 현장 기준은 **2층 B열(`2b`), 상자 마커 16**이다. `1b`는 1층 B열이다. 서비스 계정은 `large-login`을 사용한다. 입고는 대기 상태로 생성된다. 관제자가 시작하면 N11에서 양팔이 지정 칸에 보관하고, 고객 예약으로 같은 칸에서 출고한다. 카트 도착 후 패드에서 수령을 확인하면 복귀한다. 각 실행은 빈 재고로 시작하므로 재실행할 때마다 대상 칸과 양팔이 비었는지 현장에서 확인한 뒤 위 확인 옵션을 지정한다.

### 수령 후 카드 추종

패드에서 번호 35~49 중 카드를 선택하고 **수령 확인 및 추종 요청**을 누르면 관제에서 승인할 때까지 카트가 안내 지점에 대기한다. 관제에서 요청별로 승인·정지·재개·복귀를 조작한다. **수령 확인**은 수령을 저장하고 기존 복귀 흐름을 시작한다.

가상 서비스에서는 두 수령 경로를 모두 시험할 수 있다. 두 대 현장 시험에서 카트의 추종 시작·정지 근거를 확인한 뒤 현장 번들의 `fleet.json`에 `verified_follow_robot: "pinky1"`을 지정한다. 관제의 승인과 재개는 이 프로필과 현재 스마트카트 역할을 대조한다.
