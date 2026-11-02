# roport_smartcart

스마트카트(Pinky3, 문서상 pinky_764e · PP_26091 · RAM 8 GB), 담당 강지원.
[구현 계획·강사 설정 복구 기록](IMPLEMENTATION_PLAN.md), [FFT1-83](https://fp-team1.atlassian.net/browse/FFT1-83).

## 현재 범위

2026-11-02 사용자 요청 기준: **라인 상차 → 적재 후 대기 복귀 → 관제 호출 → Nav2 배송 → 하차 확인 → 복귀**.
TS-3의 기존 QR·고객 추종 시나리오와 차이가 있으며 관제 연결 전에 계약을 맞춰야 한다.

이 패키지는 첫 구현의 검증 가능한 구성요소를 제공한다. 자동 주행 노드·실기 launch·MQTT 임무 실행기는 없다.

| 파일 | 구현 |
|---|---|
| `mission.py` | 순수 임무 상태 전이, 명시 적재·하차 확인, 늦은 결과 차단, 실패/취소 시 적재 유지 |
| `goals.py` | map 프레임 실측 목표 검증, 지도 YAML·이미지 해시 변경 시 거부 |
| `navigation.py` | 목표 제출·수락·취소·결과 생명주기, 취소/timeout 중 소유권 유지 |
| `nav_client.py` | ROS NavigateToPose 연결, 전송 시 좌표/지도 검증, 명시 action 이름 |
| `velocity_gate.py` | ROS 없는 속도 소유권·명령 epoch/sequence·heartbeat/센서 지연·상한 검증 |
| `config/places.yaml` | 대기점·P7·게이트·입국장 이름과 마커 ID, **좌표는 모두 미측정** |
| `config/reference/2026-09-14/` | 세션에서 찾은 강사 설정 원본 덤프. 실행용 설정이 아님 |

이 구성요소를 통합한 주행·장애물 회피·정밀 정렬·라인 경로 실행은 아직 완료하지 않았다.
Nav2 성공/취소 결과도 물리적 정차 확인이 아니며, `VelocityGate`는 드라이버 watchdog을 대신하지 않는다.
임무 모델에는 영속 기록·관제 명령 중복 방지·재시작 후 자동 복구가 없다. 재시작 경로는 `Mission.after_restart`로 복구 대기에 두어야 한다.

## 빌드·테스트

Ubuntu 24.04 / ROS 2 Jazzy. `services/roport-pinky`에서:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select roport_smartcart
source install/setup.bash
python3 -m pytest -q src/roport_smartcart/test
```

전체 Pinky CI 범위는 `bash ci/test.sh`다. 필요 패키지는 `.github/workflows/pinky-ci.yml`과 `package.xml`을 따른다.
테스트는 모터를 구동하지 않는다. 모의 ActionServer 통합 시험은 ROS domain 198과 고유 namespace를 사용한다.
생성자만 호출해도 주행을 시작하는 코드는 없다. 모든 좌표는 실제 정차 자세를 측정하기 전 사용할 수 없다.

## 연결 시 준수할 사항

- 공용 라인 코드를 복사하지 않는다. `roport_pinky_line`(PR #19)의 기체별 IR 설정을 재사용한다.
- 라인 v0의 `node`는 가로 테이프 검출이다. 목적지 ID 판별·회전·왕복 경로는 FFT1-117의 별도 의존이다.
- `Mission.complete`는 어댑터가 도착·실제 정지·출발 허가를 검증한 뒤 호출한다. `unload_confirmed`는 하차 및 복귀 허가가 모두 있어야 한다.
- `NavClient.start(places_file, map_yaml, place_id)`는 전송 때마다 `load_destination`으로 측정 여부와 지도 해시를 확인한다. 좌표의 자유 공간 여부·AMCL 수렴은 별도 검증한다.
- 라인·Nav2·수동의 속도 토픽을 분리하고 최종 구동 출력은 한 소스만 소유한다. Nav2 behavior/smoother도 중재 경로에 포함한다.
- `VelocityGate` 시간은 같은 monotonic 기준이다. 센서 callback은 오래된 ROS stamp를 먼저 검증하고 갱신한다. 수신 시각만으로 측정 데이터 최신성을 보장하지 않는다.
- 관제 payload는 기존 MQTT envelope를 사용한다. 라이브러리의 메서드 이름을 합의된 관제 명령 이름으로 간주하지 않는다.

## 실기 전 남은 항목

최종 드라이버 명령 timeout, ROS 속도 중재 노드, 실제 정지/AMCL 준비 확인, 적재 외곽·제동거리·센서 사각 실측,
지도/목적지 좌표 확정, 라인 경로·모드 전환, 관제 호출/상하차/복귀 계약과 영속 실행 기록이 필요하다.
빌드·오프라인·모의 ROS 시험 성공은 실로봇 검증 완료가 아니다.

## 로컬 검증 기록 · 2026-11-02

Ubuntu 24.04 / ROS 2 Jazzy / Python 3.12에서 `bash ci/test.sh` 통과:
기존 센서 검사 6개, 4개 패키지 빌드·import·URDF 전개, 스마트카트 테스트 79개(모의 ROS ActionServer 포함).
Node.js 24의 `npm run test:commit` 3개도 통과했다. 실제 센서·주행·제동거리와 원격 GitHub Actions는 미검증이다.
