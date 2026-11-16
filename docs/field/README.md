# 실물 적용 준비 — 2026-11-16 로컬 개발본

적용 예정: 2026-11-17. 이 문서는 **요청 기반 FLEET + 최소 MQTT v2** 기준이다.
과거 single/two trial 더미 상하차 시험과 구분한다. 실물 없이 통신·실행부·오류 처리를
검증했으며 실제 모터, 도킹 오차, 센서, 캘리브레이션은 현장에서 확인한다.

- [Pinky 적용](pinky.md): 코드 설치 → ROS 준비 → MQTT endpoint → 상태 점검.
- [OMX 적용](omx.md): 기존 장치 소유자 연결 → 패널 API → MQTT Host → 상태 점검.
- 주요 경로: N01/OMX3, N21/OMX1, N11/양팔 공동, N22/OMX2, N12/주행 모드 전환.
- 작은 물품은 a/c열, 큰 물품은 b열. 명령 수신·팔 복귀만으로 물품 안착을 확정하지 않는다.

## 1. 관제 PC에서 배포 폴더 만들기

통합 시험 안내의 브랜치를 clone한 작업공간을 사용한다.
**main을 pull하는 것만으로 이 개발 버전이 설치되지 않는다.**
아래 도구가 현재 저장된 소스·지도·Pinky별 설정을 일반 폴더로 복사한다. ZIP은 필요 없다.
소스·설정 변경 시 새 폴더를 만들어 전달하고 기존 실행 이력은 유지한다.

```bash
cd /path/to/roport-control-test/services/roport-control-server
uv sync --locked
export HANDOFF="$HOME/roport-field/handoff-$(date +%Y%m%d-%H%M%S)"
uv run python tools/prepare_field.py --output "$HANDOFF"
cd "$HANDOFF"
sha256sum -c SHA256SUMS
```

`manifest.json`은 기준 HEAD, 로컬 수정 여부, 배포 파일별 해시를 담는다. HEAD만 같다고
로컬 변경까지 같지는 않으므로 해시로 대조한다. 관제는 이 스냅샷을 만든 checkout을 계속 사용한다.
관제 소스도 저장소 루트에서 `sha256sum -c "$HANDOFF/CONTROL_SHA256SUMS"`로 대조한다.
차이가 있으면 변경 내용을 확인하고 배포 폴더를 다시 생성한다.

| 파일 | 용도 |
| --- | --- |
| `fleet.json` | 관제의 지도·속도·골·OMX 모델/캘리브레이션 설정 후보 |
| `pinky1/trial.json`~`pinky3/trial.json` | 해당 기체용 드라이버 설정. 파일명이 trial이어도 전송은 v2 |
| `maps/` | 동일 YAML·이미지·실측 layout |
| `pinky_mqtt/`, `pinky-src/` | 이 버전의 MQTT 모듈·Pinky ROS 팀 패키지 |
| `omx/`, `manuals/` | OMX 팀 코드·이 매뉴얼. LeRobot 원본·가중치·개인 보정은 미포함 |

원본 스냅샷은 보존하고, 현장용 사본을 각 PC의 `~/roport-field/current`에 둔다.
복사 직후 해시를 검사한 다음 해당 사본의 현장 설정을 편집한다. 설정 편집 후 해시 차이는
예상되며 어떤 파일을 바꿨는지 기록한다. `.env`, 조작 키, DB 비밀번호, 영상은 Git에 올리지 않는다.

## 2. 현장에서 채울 공통 값

1. `broker.env.example`을 `broker.env`로 복사하고 **현재 관제 브로커 PC의 LAN IP**를 입력한다.
   `ip -br address`로 현재 브로커 PC의 주소를 확인한다.
   Pinky IP는 SSH/파일 전송용이며 MQTT 명령의 목적지는 `pinky1/2/3` 토픽이다.
2. 시간 동기화: 각 PC에서 `timedatectl show -p NTPSynchronized --value` 확인.
   명령·상태는 시각 차이에 민감하다. 오래된 메시지를 허용하도록 판정값을 완화하지 않는다.
3. 지도·현재 출발 위치·고객 골·N12 복귀 골·독립 정지 감시를 확인한 후 해당
   `fleet.json` 및 참여 Pinky `trial.json`의 `field_verified`만 `true`로 바꾼다.
   생성기는 자동 승인하지 않는다. 골은 현재 맵 기반 후보이며 실제 통행 가능 위치를 확인한다.
4. `fleet.json.omx`의 실제 `model_version/calibration_version`을 장치 factory 값과 맞춘다.
   `null` 또는 `sim-model/sim-cal`을 현장 확인값으로 사용하지 않는다.
5. 실제 라인 상한은 0.06m/s, 가상 기본은 0.08m/s다. `node_clearance_m`는 실물 생성본에
   넣지 않는다. 적재물까지 포함한 간격 확인 전에는 기존 보수적 구간 보호를 사용한다.

동일한 기체에 과거 trial 실행부·Fleet 실행부·가상 endpoint를 동시에 연결하지 않는다.

## 3. 브로커와 관제 실행

이미 팀 브로커가 있으면 같은 것을 사용한다. 중복으로 시작하지 않는다.
전용 시험 브로커가 필요하면 아래 예시의 바인딩 주소를 **관제 PC 자신의 LAN IP**로 바꾼다.
인증 없는 구성은 격리된 시험 LAN 전용이다. 외부 공개에는 쓰지 않는다.

```conf
listener 18883 REPLACE_WITH_CONTROL_LAN_IP
allow_anonymous true
persistence false
```

```bash
# 설치된 mosquitto가 없으면 이 관제 PC의 기존 검증 런타임 사용
export MOSQUITTO_BIN="$(command -v mosquitto || true)"
if [ -z "$MOSQUITTO_BIN" ]; then
  export MOSQUITTO_BIN="$HOME/.cache/roport-test-runtime/ubuntu-noble/root/usr/sbin/mosquitto"
  export LD_LIBRARY_PATH="$HOME/.cache/roport-test-runtime/ubuntu-noble/root/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
"$MOSQUITTO_BIN" -c "$HOME/roport-field/mosquitto.conf"
```

관제 터미널에서 공통 환경을 잡는다. 아래 `ROPORT_DATABASE_URL`에는 준비된 **관제 전용**
PostgreSQL DB를 사용한다. 운영 파일 journal로 대체하지 않는다. 기존 데이터를 삭제/초기화하지 않는다.
DB가 없으면 DB 담당자가 생성·접속 정보를 준비해야 한다. 서비스 DB를 그대로 지정하지 않는다.

```bash
cd /path/to/roport-control-test/services/roport-control-server
export FIELD_DIR="$HOME/roport-field/current"
source "$FIELD_DIR/broker.env"
export ROPORT_DATABASE_URL='postgresql://USER:PASSWORD@127.0.0.1:5432/CONTROL_DATABASE'
export ROPORT_FLEET_SOCKET="$FIELD_DIR/fleet.sock"
export ROPORT_MAP="$FIELD_DIR/maps/airport_map.yaml"
export ROPORT_MAP_LAYOUT="$FIELD_DIR/maps/layout.yaml"
export ROPORT_TRIAL_TOKEN="$(python3 -c 'import secrets; print(secrets.token_urlsafe(24))')"
# 새 관제 DB 준비/승인된 스키마 갱신 시 한 번. 실행중인 DB에는 임의 적용하지 않는다.
uv run python -m roport_control_server.adapters.db.service_tasks
uv run python -m roport_control_server.executor.fleet_runtime --config "$FIELD_DIR/fleet.json" --socket "$ROPORT_FLEET_SOCKET" --hardware-authorized
```

별도 터미널에 동일한 `ROPORT_FLEET_SOCKET`, 지도 경로, **같은** 조작 키를 설정하고 실행한다.
키를 터미널 간 복사할 때 외부 로그/공유 파일에 포함하지 않는다.

```bash
uv run uvicorn roport_control_server.adapters.api.app:app --host 127.0.0.1 --port 8000
```

`http://127.0.0.1:8000/ui/?mode=fleet`에 접속해 키를 입력한다. 실행부 시작만으로 움직이지 않는다.
기체 상태 확인과 역할 설정을 마친 뒤 별도로 `요청 처리 시작`을 누른다.
외부 PC는 우선 SSH 포워딩 `ssh -L 8000:127.0.0.1:8000 USER@CONTROL_IP`으로 접속한다.
실물 원격 제어를 위해 HTTP 서버를 LAN 전체에 임의 공개하지 않는다.

이 단계는 운영자 UI 요청만으로도 시험할 수 있다. 서비스 WSS를 포함할 때는 기존
`services/roport-control-server/docs/service-link.md`(관제 checkout)의 URL·토큰·site 설정을
별도로 맞춘다. 개발용 고객/카트 인증은 공개 배포용이 아니다.

## 4. 움직임 없는 점검 → 단계별 시험

관제 요청 처리를 끈 상태에서 실제 endpoint를 준비한다. static 검사와 MQTT 상태를 구분한다.

```bash
uv run python tools/check_field.py --bundle "$FIELD_DIR" --targets pinky1 omx3 --output "$FIELD_DIR/settings-check.json"
uv run python tools/check_field.py --bundle "$FIELD_DIR" --targets pinky1 pinky2 pinky3 omx1 omx2 omx3 shelf-01 --nav2 pinky3 --listen --seconds 5 --output "$FIELD_DIR/mqtt-check.json"
```

연결한 대상만 `--targets`에 넣는다. 이 검사는 **발행 명령 0개**, 상태 수신 전용이다.
MQTT 프로토콜 PUBACK만 보내며 `event_stored`·주행 유지·시작·정지 명령을 보내지 않는다.
`settings_ok`는 구성, `mqtt.ok`는 관측된 정지/유휴 상태다. 둘 다 성공해도 모터·센서 보정
검증 완료를 뜻하지 않는다. OMX 모델 버전은 상태 메시지에 없으므로 정적으로 대조하고 실제
명령 접수 때 Host가 다시 검사한다. 누락·false를 수동 true로 조작하지 않는다.

시험 순서: Pinky 단독 구간 → OMX 무부하/저위험 단독 작업 → 실제 입고 1건 → 출고·고객 수령 →
N12/Nav2 왕복 → 공동 팔 → 여러 기체 동시 주행. FLEET에는 OMX 완료 더미 우회가 없다.
팔 연결 전 주행만 확인하려면 기존 trial 구간 시험을 별도 실행하고 Fleet는 종료한다.

정지/오류 발생 시 물리 정지를 먼저 확인하고 관제 UI의 `중단·복구 확인`에서 기록을 대조한다.
HOLD 해제와 실행 승인은 별개다. 재시작은 같은 v2 journal/DB를 사용한다. 기존 이력 삭제나
새 파일로 실패 상태를 우회하지 않는다. 구형 v1에서 v2로 최초 전환할 때만 별도 v2 journal을 만든다.

저장할 기록: 사용한 manifest, 변경한 설정, 점검 JSON, 관제/로봇 로그, 발생 시각·물품·명령 ID.
패스워드·조작 키는 제외한다. 모든 실행 터미널에서 Ctrl+C로 종료한 뒤 장비 정지를 확인한다.
