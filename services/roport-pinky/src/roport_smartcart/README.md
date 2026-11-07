# roport_smartcart

스마트카트 담당 강지원. [구현 계획·강사 설정 복구 기록](IMPLEMENTATION_PLAN.md), [FFT1-83](https://fp-team1.atlassian.net/browse/FFT1-83).
업무 범위는 **라인 상차 → 적재 후 대기 복귀 → 관제 호출 → Nav2 배송 → 하차 확인 → 복귀**다.
QR·고객 추종은 현재 구현 범위에 넣지 않는다.

## 운행 흐름 · 2026-11-05 사용자 확인

**H1 또는 H2에서 출발 → N11 상차 → 관제가 배정한 빈 H1/H2로 복귀 → 적재 대기 → 고객 요청의 G1/G2/G3로 배송**한다.
대기칸 점유·예약과 고객 배정은 관제가 맡는다. 출발했던 칸이 계속 비어 있다고 가정하지 않으며,
로봇은 유효한 배정 없이 대기칸에 진입하지 않는다. 빈칸이 없으면 정지 상태를 보고하고 관제 지시를 기다린다.
이 배정/점유 연결은 아직 구현 전이다.

| 도면 위치 | 코드의 place ID | 라인 노드 | 도면 기준 ArUco ID |
|---|---|---|---|
| H1 / P1 대기 | `wait_p1` | H1 | 1 |
| H2 / P2 대기 | `wait_p2` | H2 | 2 |
| N11 / P7 상차 | `load_p7` | N11 | 7 |
| G1 배송 | `gate1` | 없음 · Nav2 | 8 |
| G2 배송 | `gate2` | 없음 · Nav2 | 9 |
| G3 배송 | `gate3` | 없음 · Nav2 | 10 |

현재 그래프에서 상차 경로는 H1→N09→N24→N19→N11 또는 H2→N08→N09→N24→N19→N11이며,
상차 후에는 배정받은 대기칸까지 반대 순서로 돌아간다.
고객 호출 후 H1→N09→N08→N12 또는 H2→N08→N12로 이동하고, N12에서 라인 구동을 해제한 뒤 Nav2로 게이트에 간다.
그래프 경로 검증과 실제 라인↔Nav2 전환 구현은 구분한다.

사용자는 각 장소의 마커가 부착돼 있음을 확인했다. 게이트에서는 map 기준 로봇 정차 자세까지 Nav2로 이동하고
해당 마커 ID로 장소를 확인한다. 정밀 정렬이 필요하면 카메라 보정·TF·목표 거리/방향을 측정한 뒤 마지막 접근을 연결한다.
벽의 마커 좌표는 로봇 중심 정차 좌표가 아니며, 마커 검출은 상차/하차 완료 신호도 아니다.
G1 앞 단일 Nav2 정차는 아래 현장 시험에서 확인했다. 마커 인식·정렬과 반복 정차 정밀도는 아직 검증하지 않았다.

## 현재 구현

| 파일 | 역할 |
|---|---|
| `mission.py` | 명시 적재·하차 확인, 임무 전이, 실패/취소 시 적재 유지, 늦은 결과 차단 |
| `goals.py` | 환경(sim/real)·측정 여부·map 프레임·지도 YAML/이미지 해시 검증 |
| `navigation.py`, `nav_client.py` | NavigateToPose 제출·수락·취소·결과, timeout 중 소유권 유지 |
| `navigate.py` | 사용자가 실행하는 단일 목적지/선택 복귀 CLI, 센서·위치·정지 준비 검사 |
| `profiles.py`, `config/nav2_overrides.yaml` | 공통 Nav2 설정 병합, sim/real clock·footprint·속도 분리 |
| `launch/navigation.launch.py` | sim: Gazebo+Nav2, real: 명시 설정으로 Nav2 실행. 목표를 보내지 않음 |
| `velocity_gate.py` | ROS 없는 속도 소유권·epoch/sequence·지연·상한 검증 |
| `trial_contract.py` | 기존 trial-v1 키트에 연결할 Nav2 목표·실측 완료·실패 hook 검증 |
| `config/places.yaml` | 실기 목적지. `environment: real`, **모두 미측정이므로 실행 거부** |
| `config/places_sim.yaml` | v14 지도에 대조한 **시뮬레이션 전용 후보 좌표** |
| `config/reference/2026-09-14/` | 강사 설정 원본 덤프. 자동 로딩하지 않음 |

관제 시험의 MQTT·라인/Nav2 전환·실측 완료 응답은 최신 main의
[`roport_pinky.trial_driver`](../roport_pinky/README.md)를 공통 실행부로 사용한다.
`trial_contract.py`는 순수 계약 검사이며 별도 MQTT 실행부로 연결하지 않는다.
스마트카트의 상차 후 H1/H2 복귀·적재 대기·고객 호출 전체 임무는 아직 통합하지 않았다.
`navigate`는 명시 목표를 보내는 도구이며 상차·하차·관제 임무 실행기가 아니다.
2026-11-05 Gazebo에서 G1/G2/G3→home 각각 왕복·연속 정지를 확인했다.
11/7 관제 시험에서는 H1→N11 상차 대기→N12 라인 도착·Nav2 전환까지 확인했다. 이후 Nav2 목표는 실패했으며 게이트 배송 완료로 처리하지 않는다.

### 11/7 관제·공용 라인 연결 기준

최신 main `c7c5d0e`와 민지님 PR #32·#33·#38·#39의 라인 주행·설정·지도 v14·마커 변경을 대조했다. 공용 라인 노드와 MQTT 실행부를 복제하지 않고 기존 `roport_line_nav`·`roport_pinky.trial_driver`를 사용한다.

| 연결 | 따르는 코드·조건 |
|---|---|
| 관제 명령·결과 | `roport_control_server/executor/pinky_trial.py`와 배포 키트의 trial-v1 스키마. 역할 smartcart/pinky3, binding/boot·지도/그래프 버전 일치 |
| 라인 주행 | `line_nav/command`·`line_nav/state`. N11 loading·N12 handoff와 실제 정지를 관측해서 결과 보고 |
| N12 인계 | `mode_switch.requested`로 명시 전환. Nav2·위치 추정 준비 확인 후 `navigate.requested`의 map 좌표로 주행 |
| 전체 배송 | 현재 관제 시험은 N11→N12→단일 `PUBLIC_SPACE_TRIAL` 목표다. H1/H2 적재 대기 복귀·고객 호출·G1/G2/G3 선택은 후속 계약 연결 대상 |

사용 중인 `2026-11-06-dual-web-trial-map-v14` 키트에는 main에 없는 상태 오류 보고 검사·MQTT 수신 시각/발행 보완이 일부 포함돼 있다. main의 통신 파일로 덮어쓰거나 각 PC에서 공유 JSON을 다시 생성하지 않는다. 이 키트의 manifest와 실제 validator를 기준으로 호환성을 검사한다.

### 11/5 현장·시뮬레이션 결과

| 시험 | 관측 결과 | 남은 확인 |
|---|---|---|
| 실물 H1 앞쪽20cm 상대 목표 | Nav2 성공, odom 약17.9cm, TF 기준 목표 거리오차 약2.0cm, 연속 정지 | 줄자 기반 오차·제동거리 |
| 실물 H1→G1 앞 | 입구를 돌아 odom 약4.001m 이동, Nav2 SUCCEEDED(error_code=0), TF 기준 거리오차 약2.32cm·방향오차 약6.67°, 연속 정지 | 반복 주행·실물 복귀·정밀 정렬 |
| Gazebo G1/G2/G3→home | 세 게이트 모두 성공·정지, 복귀 오차 약5.10/5.48/5.87cm(허용치6cm) | 움직이는 장애물·관제 임무 통합 |

실물은 H1에서 N09를 보는 자세로 초기 위치를 맞추고, 현장에서 G1 마커 정면의 벽 앞30cm 목표 지점을 확인한 뒤 시험했다.
선속도 상한0.03m/s, 각속도 상한0.15rad/s이며 실측 외곽과 로봇 기준점 위치를 고려한 footprint를 사용했다.
단독 Nav2 목표를 감독하며 시험했고, 완료 뒤 현장 사진으로 게이트 전방 정지를 확인했다.
거리·방향 오차는 최신 map→base_footprint TF와 목표의 차이다. 줄자로 측정한 절대 오차 또는 마커 검출·정렬 성공을 뜻하지 않는다.
현장 원본 사진·상세 로그·개별 실기 설정은 공개 저장소 밖에 보관한다. 현재 `places.yaml`은 배포용 목적지의 실측 등록·반복 검증 전 상태를 유지한다.
상차·대기 복귀·호출·라인/Nav2 전환·MQTT를 모두 연결한 시험과 사람 회피는 아직 진행하지 않았다.

관련 변경은 [PR #37](https://github.com/kernel-robot-8th/kernel-robot-repo-1/pull/37)에서 검토한다.

## 빌드·테스트

Ubuntu 24.04 / ROS 2 Jazzy, `services/roport-pinky`에서 실행한다.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to roport_smartcart
source install/setup.bash
python3 -m pytest -q src/roport_smartcart/test
```

전체 CI 범위는 `bash ci/test.sh`, 의존성은 `package.xml`과 `.github/workflows/pinky-ci.yml`을 따른다.
단위·모의 ROS 시험은 실주행 검증이 아니다. 모의 ActionServer 시험은 domain 198과 고유 namespace를 쓴다.

## 시뮬레이션

### 이 PC에서 바로 시작

실물 Pinky 연결 없이 저장소의 `services/roport-pinky`에서 실행한다.

```bash
./scripts/smartcart_sim.sh
```

Gazebo 공항·Pinky·Nav2·웹 지도 화면을 함께 시작한다. 브라우저에서 **http://127.0.0.1:8081**을 연다.
화면에 `Gazebo 시뮬레이션` 표시가 붙으며, 기존 실물 웹 화면(8080)과 구분한다.
초기 위치는 자동 설정되며 목표는 자동으로 보내지 않는다. Gazebo는 일시정지를 해제한 상태로 둔다.
이 통합 시뮬레이션의 Nav2 속도 상한은 직진 0.12m/s·회전 0.45rad/s다. 다른 값은 `ros2 launch roport_smartcart simulation.launch.py max_linear:=0.08 max_angular:=0.5`처럼 명시한다. 라인 센서·상차·N12 구동권 전환은 이 Gazebo Nav2 실행에 포함되지 않는다.
Gazebo에서 동일한 초기 입자가 만드는 미세한 음수 분산은 sim에서만 `-1e-12`까지 부동소수점 오차로 처리한다. 실물의 분산 하한 0과 양쪽의 불확실성 상한·신선도·정지 기준은 유지한다. Gazebo 창을 닫아도 서버·Nav2는 유지하며, 전체 종료는 시작 터미널의 Ctrl+C로 수행한다.
웹 지도의 `목표` 모드에서 클릭·드래그하면 목적지·방향을 지정할 수 있고 웹 정지 버튼으로 목표를 취소할 수 있다.

두 번째 터미널에서 같은 폴더로 이동한 뒤 게이트 왕복을 실행한다.

```bash
./scripts/smartcart_sim.sh gate1  # G1 → 시작 위치 복귀
./scripts/smartcart_sim.sh gate2  # G2 → 시작 위치 복귀
./scripts/smartcart_sim.sh gate3  # G3 → 시작 위치 복귀
```

한 번에 명령 하나만 실행한다. 시작 위치만 가려면 `home`, 창 없이 실행하려면 시작 명령에 `headless`를 사용한다.
목표 실행 터미널의 Ctrl+C는 현재 목표를 취소한다. 주행이 멈춘 뒤 시작 터미널에서 Ctrl+C로 전체 시뮬레이터를 종료한다.
수정 후에는 ROS 환경에서 `colcon build --packages-up-to roport_smartcart`로 다시 빌드하고 시작한다.
스크립트는 domain198·LOCALHOST를 고정하고 이전 터미널의 원격 discovery 설정을 지운다.
게이트 후보는 시뮬레이션용이고, 라인 상차·마커 정밀 접근·관제 임무 전체를 재현한 것은 아니다.

이 PC에는 웹 콘솔용 Python 환경을 준비해뒀다. 다른 PC에서 필요하면 ROS 환경을 source한 뒤
`python3 -m venv --system-site-packages /원하는/경로`와 해당 환경의 `python -m pip install Flask`로 준비하고
`SMARTCART_PYTHON=/원하는/경로/bin/python ./scripts/smartcart_sim.sh`로 지정한다.
기본 실행은 RViz를 함께 띄우지 않는다. 이 PC에서 발생한 RViz 지도 shader 오류가 웹 지도에는 영향을 주지 않기 때문이다.
RViz가 필요하면 같은 Python으로 `ros2 launch roport_smartcart simulation.launch.py rviz:=true`를 별도 시작 환경에서 사용한다.

공통 launch는 기본 `mode:=sim`, ROS domain **198**, discovery **LOCALHOST**다. Gazebo partition도 분리한다.
실기 bringup은 실행하지 않으며, 벽에서 회전할 공간이 있는 시작 위치 `(0.3,-0.3, yaw=0)`를 Gazebo·AMCL·sim home에 동일하게 설정한다.
launch만 켜면 배송 목표를 보내지 않는다.

```bash
# 터미널 1: GUI 없는 Gazebo+Nav2. 창이 필요하면 gui:=true
ros2 launch roport_smartcart navigation.launch.py mode:=sim gui:=false
```

별도 터미널에서 같은 워크스페이스를 source한 뒤 설정만 검사한다. `--check-only`는 ROS를 초기화하거나 목표를 보내지 않는다.

```bash
SMARTCART_SHARE=$(ros2 pkg prefix --share roport_smartcart)
AIRPORT_SHARE=$(ros2 pkg prefix --share roport_gz_world)
ros2 run roport_smartcart navigate --mode sim \
  --places "$SMARTCART_SHARE/config/places_sim.yaml" \
  --map "$AIRPORT_SHARE/maps/airport_map.yaml" --target gate1 --check-only
```

목표 송신은 사용자가 다음 명령을 별도로 실행할 때 시작한다. `--return-to home`은 첫 목적지 성공과 연속 정지 확인 뒤 두 번째 목표를 보낸다.
`home`은 시뮬레이션 시작점이며 실기 충전 칸을 뜻하지 않는다.

```bash
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ros2 run roport_smartcart navigate \
  --mode sim --domain-id 198 \
  --places "$SMARTCART_SHARE/config/places_sim.yaml" \
  --map "$AIRPORT_SHARE/maps/airport_map.yaml" \
  --target gate1 --return-to home --timeout 120
```

`places_sim.yaml`의 `measured: true`는 시뮬레이션 fixture를 허용하는 값이다. 후보 좌표를 실측했다는 뜻이 아니며,
Gate 후보는 제안된 벽 마커 위치 앞 0.30 m다. `environment: sim`을 바꾸거나 이 좌표를 실기에 복사하지 않는다.

## 실기

기체·적재물 외곽과 센서 가림·제동거리·주행 상한을 측정하고 현장 정지 수단을 확인한 뒤 사용한다.
launch는 모터 bringup을 실행하지 않는다. 센서/odom 구동은 현장 절차를 따르며, 실기 초기 위치는 운영자가 설정한다.
실기 모드는 초기 위치를 자동으로 `(0,0,0)`에 맞추지 않는다.

실기 launch에는 **지도·도메인·측정 footprint·두 속도 상한**을 모두 명시해야 한다.
아래 변수는 현장에서 확인한 값으로 먼저 설정한다. footprint는 로봇 기준 미터 단위 볼록 다각형 JSON,
속도는 m/s·rad/s다. sim 기본값은 실기 설정으로 승인된 값이 아니다.

```bash
ros2 launch roport_smartcart navigation.launch.py mode:=real gui:=false \
  map:="$ROPORT_MAP" ros_domain_id:="$ROPORT_DOMAIN" \
  footprint:="$ROPORT_FOOTPRINT" max_linear:="$ROPORT_MAX_LINEAR" \
  max_angular:="$ROPORT_MAX_ANGULAR"
```

실제 로봇 중심의 정차 자세를 측정해 `places.yaml`의 해당 pose/측정 표시와 지도 해시를 확정한 뒤에만 실행한다.
현재 제공된 실기 좌표는 미측정이므로 검사와 실행이 실패하는 것이 정상이다.

```bash
ros2 run roport_smartcart navigate --mode real --domain-id "$ROPORT_DOMAIN" \
  --places "$ROPORT_PLACES" --map "$ROPORT_MAP" --target gate1 --check-only
# 설정·현장 준비 확인 후 동일 명령에서 --check-only를 제거하면 실제 목표를 보낸다.
```

## 준비·취소·연결 조건

- CLI는 매 목표 직전 환경/지도 해시/좌표를 다시 검사한다. scan·odom·AMCL·map→base_footprint TF와 연속 정지가 필요하며 sim clock도 실제로 증가해야 한다.
- `map_server/map` GetMap 응답의 frame·크기·해상도·원점 자세·점유 셀을 로컬 지도와 대조한다. 매 목표 전 새 응답을 요구하고 주행 중에도 최대 1 Hz로 확인한다. 서비스 부재·응답 실패·지도 불일치·확인 후 2초 초과 시 시작을 거부하거나 진행 목표를 취소한다. 정적 지도의 header timestamp는 수신 최신성 기준으로 쓰지 않는다.
- AMCL 공분산은 유한하고 대각 성분이 음수가 아니어야 하며 x/y 분산 각각 0.04 m² 이하, yaw 분산 0.1 rad² 이하를 요구한다(공용 trial_driver 기준). 최신 추정이 기준을 벗어나면 이전 정상 샘플도 즉시 무효화한다. frame·위치·단위 quaternion 검사도 적용하며, 이 분산 기준을 실제 정차 정확도로 간주하지 않는다.
- 진척 검사는 `PoseProgressChecker`로 병진 0.05 m 또는 회전 0.15 rad를 인정하고, 두 움직임 모두 없는 15초 정체는 실패 처리한다. 0.15 rad/s 제자리 저속 회전을 병진 정체로 오판하지 않도록 구성했다.
- 준비 대기와 주행 중 `request_nomotion_update`로 AMCL 재관측만 요청한다(최대 1 Hz, 이전 요청 진행 중이면 중복 금지). 초기 위치를 재설정하거나 모터 명령을 보내지 않는다.
- 기본 준비 대기는 20초(`--startup-wait`), 목표 timeout은 120초다. timeout/Ctrl+C에는 Nav2 취소를 요청하고 terminal result까지 기다린다. 결과가 없으면 소유권을 해제했다고 간주하지 않는다.
- Nav2 결과는 실제 정차 증거가 아니다. 다음 목표 전에 최신 odom의 연속 정지를 확인한다. CLI는 raw `cmd_vel`을 발행하지 않는다.
- 공용 라인은 `roport_line_nav`의 `line_nav/command`와 `line_nav/state`를 연결한다. `start/stop/load_done/speed`, `loading/arrived/handoff/error/stopped`를 사용하고 **N12 handoff**에서 Nav2로 전환한다. 관제 시험 어댑터는 `roport_pinky.trial_driver`에 구현돼 있으며 현장 연동 검증은 별도다.
- 라인·Nav2·수동 출력은 최종 구동에서 단일 소유권으로 중재해야 한다. behavior/smoother도 포함한다. 순수 `VelocityGate`와 라인 `cmd_vel_guard`는 최종 드라이버 watchdog을 대신하지 않는다.
- `unload_confirmed`는 하차와 복귀 허가를 모두 검증한 어댑터만 전달한다. 관제 payload·식별자·영속 중복 방지는 기존 공통 실행부 계약을 따른다.

실기 목적지·센서 사각·장애물 대응·라인 전환·관제 통합·프로세스 종료 시 구동 정지는 후속 검증 항목이다.

## 관제 메시지 연결 · FFT1-90

[통신 명세 v5](https://fp-team1.atlassian.net/wiki/spaces/MFS/pages/17530882)는 현재 시험에
[Pinky 주행 시험 가이드 v2](https://fp-team1.atlassian.net/wiki/spaces/MFS/pages/19988482)의 `roport-pinky-driving-trial-v1` 키트를 지정한다.
저장소의 일반 `schema_version: 1.0` 예시나 goal_id만 있는 운영 검토안과 혼합하지 않는다.
배포 키트의 MQTT runner가 envelope·binding/boot·중복·accepted를 처리하고 ROS driver hook을 호출한다.

- `navigation_destination`: 키트의 `validate_trial`을 주입하고 mode·map/goal 버전·좌표·명령 신선도·만료를 검증한다.
  `expires_at`이 남아 있어도 시험 계약상 `sent_at`이 3초보다 오래되면 거부한다. real은 현장 검증된 profile이 필요하다.
- `navigation_completed`: Nav2 성공·정지·라인 해제·독점 Nav2 소유권·지도 일치와 최신 측정 pose의 허용오차를 확인한 뒤
  `(원본 command, 'completed', details)`를 만든다. 요청 pose를 결과로 복사하지 않는다.
- `navigation_failed`: `(원본 command, 'failed', {'error_code': ...})`를 만든다. 상태 보강·정지·실제 발행은 runner와 ROS driver 책임이다.

이 모듈은 순수 계약 함수다. 실제 MQTT 발행과 ROS driver hook은 공통 `roport_pinky.trial_driver`와
저장소의 새 관제 키트가 담당하며, 이 모듈을 두 번째 실행부로 붙이지 않는다.
새 키트의 heartbeat/close hook이 필요하므로 기존 Confluence ZIP을 그대로 재사용하지 않는다.
관제 시험은 N11→N12→공용공간 골까지이며 H1/H2 적재 대기 복귀·G1/G2/G3 배송 전체 임무와 구분한다.
계약에 없는 `navigate.progress`를 추가하지 않고, 순수 검사 결과를 실제 N12 전환 완료로 보고하지 않는다.
검증은 합성 데이터 단위 시험과 배포 키트의 실제 `validate_trial`/`check_result` 대조로 수행했다.
원본 배포 키트·인증 파일은 저장소에 복사하지 않는다.

관제 trial과 GUI를 함께 쓸 때는 GUI에 `teleop_enabled:=false`를 지정해 직접 구동 발행자를 없앤다.
이때 웹 정지·수동 조종·별도 목표/경로 명령은 사용할 수 없으므로 현장 정지 수단을 유지한다.
trial launch에는 실기 외곽·속도·`set_initial_pose: false`를 반영한 `params_file`을 명시하고,
기존 smartcart Nav2/라인 launch와 중복 실행하지 않는다. 시험용 기본 footprint를 실측값으로 간주하지 않는다.
지도 식별자는 trial의 YAML+이미지 합성 SHA256 앞16자와 로컬 목적지 파일의 파일별 SHA256을 구분한다.

후속 순서는 FFT1-92 실물 센서/정지 확인 → FFT1-83 단일 목표 → FFT1-81 N12 구동권 전환 →
FFT1-84 관제 호출·도착/실패·복귀 연결 → FFT1-85 통합 시험이다. 사람 감속·정지·우회는 FFT1-120에서 진행한다.

## 11/6 관제 후보 키트 배포 확인

전달받은 `2026-11-06-dual-web-trial` 후보 키트의 manifest 23개 파일을 대조하고,
실제 핑키의 별도 시험 경로에 설치했다. 공용 ROS 패키지 3개 빌드와 Paho MQTT 2.1.0 설치,
지도 식별자·경로 5개·양쪽 역할 설정·미검증 실기 차단·guard 부재 시 구동 거부를 확인했다.
실제 ROS 관측은 H1/line/정지, Nav2 및 위치 추정 준비 상태였으며 주행 명령은 보내지 않았다.

이 키트의 2대 시험은 H1→N11 라인 합류 후 N11→N12 라인 주행, N12 인계 후 공용공간 시험점 이동이다.
G1/G2/G3 고객 배송 전체 임무와 구분한다. 원본 `field_verified=false`와 공유 JSON은 유지하며,
관제 브로커 연결·2대 실주행·N12 자동 전환은 아직 검증하지 않았다.
공용 trial launch에는 현장에서 확인한 `max_angular:=0.15`를 명시해 라인과 Nav2 상한을 맞춘다.

지도 v14 병합 후에는 목적지 파일의 두 지도 해시를 갱신했다. 후보 정차점은 변경된 02 인도장 받침 점유 셀에서 떨어져 있어 유지하되 시뮬레이션 지도에서 재검사한다.
위 후보 MQTT 키트는 여전히 v13 그래프/지도 설정이다. 로봇에 설치한 시험본은 `f9ecb38` 기준으로 별도 유지하며,
v14에서 연결하려면 관제와 두 기체에 같은 새 `trial.json`을 배포해야 한다. 원본 ZIP의 공유 JSON이나 현장 검증 표시는 임의로 바꾸지 않았다.
