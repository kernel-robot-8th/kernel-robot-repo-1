# 추종 서비스 전체 연결 가상 검사

`tools/verify_follow_service.py`는 실제 Java HTTP API → WSS → 관제 DB/owner → MQTT → 가상 Pinky를 통과한다. 화면 버튼과 같은 HTTP 요청을 사용하며 브라우저 DOM 검사는 별도 패드 Playwright 검사에서 수행한다. 카메라·센서·모터는 가상이고 실물 추종 성능을 검증한 것으로 보지 않는다.

전체 시나리오는 아래 명령으로 명시 실행하는 검사다. 기본 CI는 서비스·관제 단위/연결 회귀와 패드 상태 계약 브라우저 검사를 실행하며, 이 1배속 전체 주행 스크립트를 자동 실행하지는 않는다.

## 실행

Java 21, PostgreSQL 16 실행 파일, Mosquitto와 관제 Python 의존성을 준비한다. 저장소 `services/roport-control-server`에서 실행한다. `PG_BIN`은 `initdb`, `pg_ctl`, `createdb`가 있는 디렉터리이며, `MOSQUITTO_BIN`은 브로커 실행 파일이다.

```bash
uv run python tools/verify_follow_service.py \
  --pg-bin "$PG_BIN" \
  --mosquitto "$MOSQUITTO_BIN" \
  --output "/tmp/roport-follow-e2e-$(date +%Y%m%d-%H%M%S)" \
  --timeout 900
```

실행마다 새 `service_test`·`control_test`와 임의 loopback 포트를 만든다. 기존 운영 DB·브로커는 사용하지 않는다. 출력 경로는 없어야 하며 Unix socket 제한 때문에 짧은 경로를 사용한다. Java 빌드와 실제 가상 주행 때문에 수 분이 걸린다. 기본 제한은 15분이고 주행 속도는 0.08m/s, 시뮬레이션 배속은 1배로 고정한다.

## 통과 기준

1. 시작 승인 전 명령 없음 → API 입고 요청 → MQTT로 선반 입고 완료 → 고객 예약 → 스마트카트 공용공간 도착. DB에 재고·완료 상태를 직접 넣지 않는다.
2. appearance 지원 여부 조회 → 수령 API → capture → 서비스 인증 프레임 프록시에서 PNG/JPEG 확인 → 후보 select. 수령·조작의 같은 요청 재전송은 하나로 유지한다.
3. 등록 대기·운영자 승인 대기 중 상태 GET을 반복해도 pose가 고정되고 새 주행 명령이 나오지 않는다. 운영자가 명시 승인한 후에만 `following`과 로봇 active 보고가 확인된다.
4. 패드 pause → 실제 정지 보고 → GET 반복 중 정지 유지 → resume → following → return → 정지 확인 후 N12/Nav2 복귀·라인 전환·주차 완료까지 진행한다.
5. 최종 서비스 상태는 `COMPLETED`/`done`, 수령 1건, 추종 시작 2회(승인·재개), 정지 2회(일시정지·복귀), 등록 capture/select 각각 1회다.

성공하면 출력 경로의 `result.json`, `cart-commands.json`, `control-snapshot.json`에 근거를 남긴다. 실패하면 `failure.json`, timeout 시 `timeout-snapshot.json`을 함께 확인한다. `fleet/` 아래 로봇·owner·broker 로그와 DB 파일도 남는다. 끝나면 시험 프로세스와 PostgreSQL은 종료한다.

출력 폴더의 `service-env.json`, `ui-key.txt`, 실행 로그에는 시험용 토큰이 있으므로 Git에 올리지 않는다. 출력 디렉터리 권한은 700, 파일은 600이다. 실패한 시험 기록을 덮어쓰지 말고 새 출력 경로로 다시 실행한다.
