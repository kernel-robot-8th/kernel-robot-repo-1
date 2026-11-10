# roport_smartcart

스마트카트 담당 강지원. [구현 계획·강사 설정 복구 기록](IMPLEMENTATION_PLAN.md), [FFT1-83](https://fp-team1.atlassian.net/browse/FFT1-83).
업무 범위는 **라인 상차 → 적재 후 대기 복귀 → 관제 호출 → Nav2 배송 → 하차 확인 → 복귀**다.
QR·고객 추종 주행은 미구현이다. 마커 없는 대상 검출·선택·추적의 PC 시험 화면은 아래 절차로 준비했다.

## 마커 없는 대상 추적 준비 · FFT1-86

`target_preview`는 **관찰 전용**이다. ROS·MQTT·Nav2·`cmd_vel`을 연결하거나 로봇을 기동하지 않는다.
PC의 카메라 번호, 영상 파일, OpenCV에서 지원하는 스트림 URL을 입력받아 YOLO + ByteTrack 결과를 표시한다.
검출 상자를 클릭하면 한 ID를 선택한다. 대상 누락, 다른 상자와 겹침, 급격한 위치 변화,
영상 지연·종료·오류가 있으면 선택을 해제하고 **직접 재선택**해야 한다. 자동 재획득은 하지 않는다.
클릭 위치가 여러 상자에 걸쳐도 선택하지 않는다. `R`은 선택 해제, `Q`/Esc는 종료다.

저장소 루트에서 PC 전용 가상환경으로 설치한다. ROS 기본 설치와 CI에는 AI 의존성을 추가하지 않는다.

```bash
python3 -m venv .venv-target-preview
.venv-target-preview/bin/python -m pip install -e 'services/roport-pinky/src/roport_smartcart[preview]'
.venv-target-preview/bin/target_preview --source 0 --model /absolute/path/to/yolo11n.pt
# 녹화 영상: --source /absolute/path/to/test.mp4
# 인형용 추가 학습 모델: --model /absolute/path/to/doll.pt --class-id 0
```

가상환경 `.venv-*/`는 Git 제외 대상이다. 녹화 영상(`.mp4` 등)·캡처·모델은 **저장소 밖**에 보관한다.
`git add -A` 전에 `git status --short`로 확인한다. 일반 `.mp4` 확장자 전체를 무시하지 않으므로
저장소 안 임의 경로에 영상을 두면 추적 대상이 될 수 있다.

인증정보가 포함된 스트림 주소는 `--source`에 직접 쓰지 않고 `--source-env`로 전달한다.
다음은 Bash에서 셸 기록·명령 인자에 주소를 직접 남기지 않는 입력 예시다.

```bash
read -r -s -p 'Stream URL: ' PINKY_VIDEO_SOURCE; echo
export PINKY_VIDEO_SOURCE
.venv-target-preview/bin/target_preview --source-env PINKY_VIDEO_SOURCE --model /absolute/path/to/yolo11n.pt
unset PINKY_VIDEO_SOURCE
```

두 입력 옵션은 동시 지정할 수 없으며, 없는 변수·빈 값은 실행 전에 거부한다.
환경변수도 접근 권한이 있는 프로세스에서 읽힐 수 있고 OpenCV 등 외부 라이브러리의 진단 출력은 별도다.
앱은 환경변수 값을 상태 표시나 오류 메시지에 출력하지 않는다.

사용자가 확보한 **신뢰할 수 있는 로컬 검출 모델 `.pt`**를 지정한다. 가중치·현장 원본 영상은 커밋하지 않는다.
기본 class ID `0`은 COCO person 기준이며, 사용자 모델은 해당 모델의 인형 클래스 ID를 지정한다.
가중치를 자동으로 다운로드하지 않는다. 기본 CPU 추론이며 첫 추론이 느리면 선택 불가 상태로 기다린다.
Ultralytics 설치가 GUI를 지원하는 OpenCV를 함께 설치한다. 화면이 있는 PC에서 실행한다.
선택 의존성은 [Ultralytics 설치 문서](https://docs.ultralytics.com/quickstart/)와
[추적 API](https://docs.ultralytics.com/modes/track/)를 사용한다. Ultralytics 코드·모델의
[라이선스](https://www.ultralytics.com/license)는 이 패키지와 별도로 적용된다.
Ultralytics는 AGPL-3.0/Enterprise 조건을 안내한다. 선택 설치나 교육용 시연이라는 이유만으로
조건 충족을 가정하지 않는다. 현재 범위는 PC 관찰용 평가 코드이며 **팀의 시연·배포 사용 조건은 미확정**이다.
이 패키지의 `LicenseRef-Proprietary` 선언을 바꾸거나 전체 프로젝트의 AGPL 적합성을 확정하지 않았다.
시연·배포 범위, 해당 코드·모델의 조건, 대응 소스 공개 범위 또는 Enterprise 사용 여부를
팀이 FFT1-86에 기록한 뒤 배포 방식을 정한다. 충족하기 어려우면 검출기 대체를 검토한다.
모델·AI 포함 실행 환경을 기본 ROS 설치나 배포 산출물에 포함하지 않는다.

`--max-age` 기본 0.5초(최대 2초)는 **로컬 프레임 읽기 시작부터 추론 결과 표시까지**의 허용 시간이다.
캡처/추론은 별도 worker가 담당하여 읽기가 멈춰도 화면이 이전 프레임을 만료 처리한다.
카메라 자체·네트워크 버퍼의 지연이나 원본 촬영 시각을 보증하지 않으므로,
이 값은 실물 제어용 센서 timestamp/watchdog를 대신하지 않는다. 영상 파일은 원래 FPS 이하로 재생한다.

**추적 ID는 신원 확인이 아니다.** 현재 appearance Re-ID·얼굴인식은 없고,
부드러운 ID 뒤바뀜·검출 누락된 다른 대상은 이 검사만으로 판별할 수 없다.
일반 person 모델이 작은 종이 인형을 검출하는지도 미검증이다. 검출 실패 시 현장 영상으로 추가 학습을 검토한다.
거리 추정, 카메라 보정·TF, 지도 좌표 변환, 추종 Nav2 목표·구역 제한,
실제 모터 정지 및 기존 trial_guard 소유권 연결은 후속 작업이다.
이 화면의 선택 해제는 **로봇 정지 명령이 아니다**.

실물 확인 순서: 로봇 정지 상태에서 인형 2~3개를 비추고 선택 → 좌우 이동 → 서로 교차 → 가림/재등장
→ 영상 연결 해제. 검출·ID 전환·지연을 확인한 뒤 거리 추정과 저속 추종을 연결한다.
작은 인형(리뷰 기준 높이 7.8~11.8 cm)은 거리별 검출률도 기록한다.
예를 들어 20/40/60/80 cm에서 앞·뒤·옆 방향별로 10초씩 촬영하고,
화면에 보이는 평가 프레임 수·올바른 검출 수·오검출 수·ID 전환 수·처리 지연을 적는다.
검출률은 `올바르게 검출한 프레임 / 대상이 보이는 평가 프레임`으로 계산하며,
모델·confidence·해상도·조명을 함께 남긴다. 중복된 한 영상으로 학습·평가하지 않고,
거리별 결과를 보고 사용 거리와 추가 학습 필요 여부를 정한다.

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
./scripts/smartcart_sim.sh gate1  # G1 → H1 자동 복귀 데모
./scripts/smartcart_sim.sh gate2  # G2 → H1 자동 복귀 데모
./scripts/smartcart_sim.sh gate3  # G3 → H1 자동 복귀 데모
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
Gate 후보는 제안된 벽 마커 위치에서 로봇 중심까지 0.25 m다. `environment: sim`을 바꾸거나 이 좌표를 실기에 복사하지 않는다.

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
공용 trial launch의 라인 회전은 `line_rotate_speed:=0.15`, Nav2 회전은 `max_angular:=0.15`로 각각 명시한다. 라인 병진 상한은0.06 m/s, 회전 상한은0.4 rad/s 이하로 제한한다.

지도 v14 병합 후에는 목적지 파일의 두 지도 해시를 갱신했다. 후보 정차점은 변경된 02 인도장 받침 점유 셀에서 떨어져 있어 유지하되 시뮬레이션 지도에서 재검사한다.
위 후보 MQTT 키트는 여전히 v13 그래프/지도 설정이다. 로봇에 설치한 시험본은 `f9ecb38` 기준으로 별도 유지하며,
v14에서 연결하려면 관제와 두 기체에 같은 새 `trial.json`을 배포해야 한다. 원본 ZIP의 공유 JSON이나 현장 검증 표시는 임의로 바꾸지 않았다.


## 가까운 정차와 H1/H2 복귀 · FFT1-83 / FFT1-84

시뮬레이션에서 G1/G2/G3 후보를 벽 쪽으로 5 cm 옮겼다(벽→로봇 중심의 수직 거리 30→25 cm).
G3는 상단 모서리 회전 여유를 위해 마커 중심선에서 통로 쪽으로 5 cm 옮겼다. 중심선 그대로의 25 cm 후보와 사전 정렬 시도는 충돌 예측으로 실패해 기본 경로에 넣지 않았다.
차체 앞끝과 벽 사이 거리는 이 값에서 전방 차체 길이를 뺀 값이며, 적재물 외곽도 포함해 실측해야 한다.
sim 프로필은 최종 허용 오차를 위치 3 cm·방향 0.08 rad로 줄이고, 대체 목표 탐색 반경 2 cm·최종 접근 최저속도 0.02 m/s를 사용한다.
실기 프로필과 공용 trial의 허용 오차는 이번 시뮬 튜닝으로 변경하지 않았다.
`navigate` 결과의 `position_error_m`·`yaw_error_rad`는 map→base_footprint 추정 오차이며 실제 벽 거리나 실측 정차 정확도가 아니다.

게이트 마커는 DICT_4X4_50의 G1=8·G2=9·G3=10(100 mm)이다.
현재 공용 콘솔 `marker_detect`는 ID·모서리 검출과 FOV 기반 참고 거리만 제공한다.
마지막 정밀 접근에는 동일 해상도의 카메라 내부 보정/왜곡 계수, 카메라→차체 TF, 실제 마커 크기를 먼저 확정해야 한다.
그 뒤 검출 모서리의 보정된 3D 자세를 차체 기준으로 변환해 벽과의 거리·좌우·방향 오차를 계산하고,
기존 구동 소유권 안에서 저속 접근한다. 다른 ID·오래된 영상·인식 상실에는 정지하고 도착으로 처리하지 않는다.
마커 정렬 구동과 현장 거리 보정은 아직 연결하지 않았으며, 게이트 도착이나 마커 인식을 수령 확인으로 대체하지 않는다.

H1/H2는 충전 및 적재 대기 구역이다. 관제가 점유·예약을 확인한 뒤 `wait_p1`(H1) 또는 `wait_p2`(H2)를 배정한다.
순수 `Mission.assign_wait_bay(place_id, operation)`은 `WAIT_LOAD`/`WAIT_UNLOAD`에서만 이를 받는다.
`load_confirmed`/`unload_confirmed`는 배정이 있어야 복귀 단계로 진입하고,
`wait_route_finished`/`home_reached`에는 배정과 같은 `place_id`가 필요하다.
배송 출발 시 이전 배정은 폐기하므로 수령 후 새로 예약해야 하며, 도착 지점이 다르면 완료하지 않는다.
`unload_confirmed`는 기존처럼 수령 확인과 복귀 허가를 모두 검증한 어댑터 입력이다.
이것은 오프라인 상태 모델의 조건 검사다. MQTT 수령 메시지 연결·예약 유효성 확인·Nav2→라인 재획득과 실제 H1/H2 복귀 주행은 공용 실행부에 추가 연결해야 한다.


11/7 Gazebo에서 수정 후보의 G1/G2/G3 도착·연속 정지를 각각 확인했다.
지도 추정 위치 오차는 약 2.58/1.66/1.94 cm, 방향 오차는 약 0.0031/0.0166/0.0171 rad였다.
이는 각 1회 결과이며 반복 재현성·실제 벽 거리·실물 정차 정확도나 마커 정렬 성공을 뜻하지 않는다.
스마트카트 테스트 218개 통과 후 G3 변경점의 padded footprint 빈 셀 검사도 통과했다.

## LiDAR·초음파 근사 센서 시험 · FFT1-120

Gazebo 전용이다. 원본 Pinky xacro를 포함한 팀 wrapper에 전방 거리 센서를 추가한다.
모델 기준 라이다 평면은 바닥에서12.5cm, 초음파 프레임은3.7cm다. 전체 기체 높이14.2cm와 레이저 평면을 구분한다.
초음파 근사는 수평15° 다중 광선의 최단 거리이며 음향 반사·흡음·간섭·바닥 반사를 재현하지 않는다.
실물 ADC 환산식의 표현 범위를 고려해 sim 범위는0.02~0.95m로 제한했다. 실물 거리 보정값을 검증했다는 뜻은 아니다.

```bash
# 기존 시뮬레이터 종료 후, 5cm 장애물이 있는 GUI 실행(자동 주행 없음)
./scripts/smartcart_sim.sh sensors 0.05
# 별도 터미널: 장애물 뒤의 sim 목표로 명시적으로 주행
./scripts/smartcart_sim.sh sensor-goal
```

높이는 m 단위이며 `0.10`, `0.12`, `0.13`, `0.15`로 바꿔 비교할 수 있다. 높이별 재시작은 시험 초기 위치도 되돌린다.
주황 장애물은 시작점 앞 `(0.9,-0.3)`에 있고, `sensor_goal`은 `(1.2,-0.3)`이다. 실기 H1/H2 좌표가 아니다.
웹 지도는 기존처럼 `http://127.0.0.1:8081`에서 확인한다.

- `/scan`은 기존 라이다, `/sim/sonar_scan`은 낮은 수평 광선, `/us_sensor/range`는 Range 변환 출력이다.
- 시뮬의 두 costmap에 낮은 `/sim/sonar_scan`을 받는 `ObstacleLayer`를 추가해 장애물 표시·광선 지우기를 수행한다. Range 출력은 Collision Monitor에서 계속 사용한다.
- Nav2 출력은 Collision Monitor → `/cmd_vel_safe` → Gazebo bridge 순서다. 원래 `/cmd_vel`이 구동 bridge로 직접 가지 않는다.
- 후보 전방 감속 영역45cm에서는 속도를30%로 줄이고 정지 영역22cm에서는0을 보낸다. 거리는 로봇 중심 기준 영역이며 실물 제동거리 설정이 아니다.
- 라이다 또는 Range가0.5초 이상 오래되면 명령 처리 시 정지한다. 정상 관측이 돌아오면 필터가 다시 통과시킬 수 있다. 임무 복구/재출발 승인은 별도 실행부 책임이다.
- Gazebo 무반환(+inf)은 유한한 최대 거리로 변환해 센서 정상/장애물 없음과 센서 끊김을 구별한다. 너무 가까운 반환(-inf)은 최소 거리로 취급하고, NaN/범위 밖 프레임은 버려 timeout 정지를 유도한다.

실물 trial·MQTT·라인 주행에는 이번 필터를 자동 연결하지 않는다. 낮은 물체의 측후방, 센서보다 낮은 물체와 광선 사이 작은 물체는 사각이다.
실물 적용에는 초음파 거리/장착 검증과 양방향 구동·명령 watchdog을 포함한 최종 단일 구동권 검증이 필요하다.
Collision Monitor는 인증된 안전 제어기가 아니며 프로세스 종료 시 모터 watchdog을 대신하지 않는다.

2026-11-07 Gazebo 고정 위치 비교에서 5/10/12cm 장애물은 라이다 전방 반환이 약2.60m(뒤쪽 벽),
13/15cm 장애물은 약0.52~0.54m였다. 낮은 Range는 다섯 높이에서 모두 약0.513m를 반환했다.
5cm 장애물로 전진 입력0.12m/s를 주었을 때 필터 출력은0.036m/s로 감속한 뒤0으로 정지했다.
정지 확인 시 Range는0.21m 이내였으며 센서 원점과 로봇 중심의 차이 때문에 정지 영역22cm와 값이 다르다.
합성 ROS 센서 시험에서는 LiDAR 정지와 Range 수신 중단 시 정지 유지·수신 복구 후 통과도 확인했다.
이는 고정 박스 단일 시험이며 사람 추적·움직이는 장애물 회피 또는 실물 제동거리 검증 결과가 아니다.

## Gazebo H1 복귀 · FFT1-84

`home`은 공용공간의 초기 시험 위치이며 H1이 아니다. 게이트 명령의 기본 복귀는 이제 H1이다.
이미 N12 접근 범위 안이면 출구로 되돌아가지 않고 라인 획득부터 이어간다.

```bash
./scripts/smartcart_sim.sh h1     # 현재 위치에서 H1 복귀만
./scripts/smartcart_sim.sh gate2  # 공용공간에서 시작: G2 도착 후 H1 복귀
```

N12 접근점인 공용공간 출구 `return_entry`까지 기존 Nav2 클라이언트로 이동하고 실제 정지 확인 뒤,
최대45cm 이내에서 모의 위치로 N12 라인에 정렬한다. N12 → N08 → N09 → H1을 기존 `roport_line_nav.RouteExecutor`로 주행한다.
라인 구간은 0.06m/s, 회전은 0.45rad/s이며 기존 회전·후진 주차·라인 놓침 기준을 유지한다.
모든 속도는 기존 Collision Monitor를 거쳐 Gazebo로 전달한다. 센서/odom 수신 및 ROS timestamp가
0.5초 넘게 오래되면 중단하며, 주행 취소·오류·종료 때 정지 명령을 보낸다.
정리 중 중복 SIGINT·SIGTERM·SIGHUP은 무시해 정지 명령 10회와 정리를 끝낸다.
`kill -9`·OOM 또는 Collision Monitor 선행 종료 시 정지 명령이 전달되지 않아 Gazebo에 마지막 속도가 남을 수 있으며, 별도 구동 watchdog을 대체하지 않는다.

Gazebo에 실제 IR 센서는 없다. 배치 데이터의 테이프와 `/sim/gazebo_poses`의 실제 Gazebo 모델 위치로
9점 IR 근사를 만들고, N12 전환 때는 같은 모의 위치로 제한된 정렬을 수행한다.
라인 실행부의 이동 거리·회전 누적은 기존처럼 wheel odom을 사용한다.
이는 실물 IR·ArUco 전환 정렬 구현이 아니다.
H1은 v14 배치에서 world/map 약 `(1.5439,-1.6852)`이며, 주차 완료는 IR이 H1 가로선을 감지한
상태이므로 바퀴 축 중심과 가로선 중심 사이에는 센서 장착 거리만큼 차이가 있다.
관제 MQTT·H1/H2 점유 선택·실물 trial은 자동 변경하지 않는다.

Gazebo의 일시정지 버튼을 누르면 시뮬레이션 시간과 센서 관측도 멈춘다. 기존 freshness 감시가
주행을 취소하므로 재생(▶) 상태로 바꾼 뒤 명령을 다시 실행한다. 이 데모가 임의로 일시정지를 해제하지 않는다.
게이트 왕복 데모는 공용공간 안에서 시작한다. H1에서 출발하는 반대 방향 라인 전환은 이 복귀 데모 범위 밖이며,
라인 구역에서 `--gate`를 실행하면 Nav2로 대신 나가지 않고 거부한다.
공용공간 경계는 layout의 `zone03_left`·`zone03_bottom` 벽 좌표에서 읽는다.
`h1`도 공용공간·N12 접근 범위·이미 H1 주차 상태에서만 사용하며, N09 등 나머지 라인 구역에서는 Nav2 출발을 거부한다.

2026-11-07 Gazebo에서 출구 접근 Nav2 성공을 확인한 뒤 N12 근처 재시도로 라인 획득 → N08 → N09 → H1
전체 라인 복귀·180° 회전·후진 주차가 `arrived`로 완료됐다. 최종 실제 모델 위치는
`(1.532,-1.708)m`, yaw 약1.442rad로 H1 북쪽 방향과 약7.5° 차이가 있었다.
위 기록은 N12 근처 재시도 기준이다. 이후 main에서 공용공간 시작 → G2 → H1을 단일 명령으로 완주하고 최종 정지를 확인했다.
실제 IR·충전 접점 연결이나 관제 통합 완료를 뜻하지 않는다.


### 수령 확인·배정 대기칸 복귀 모의 관제 · FFT1-84

운영 관제 `executor/pinky_trial.py`는 navigate 완료에서 COMPLETE로 끝나며 수령·대기칸 예약 메시지가 없다.
아래는 **domain198/LOCALHOST 전용 ROS 모의 관제**이며 MQTT trial-v1을 확장하거나 실제 예약 DB에 연결하지 않는다.
기존 `gate1/2/3`는 수령을 기다리지 않는 자동 복귀 데모다. 수령 대기 시험에는 `delivery1/2/3`를 사용한다.

```bash
# 터미널 1: 공용공간에서 시작하는 Gazebo (실물 연결 없음)
./scripts/smartcart_sim.sh
# 터미널 2: 다른 로봇이 H1을 점유했다고 가정. 둘 다 비어 있으면 --occupied 생략
./scripts/smartcart_sim.sh control --occupied H1
# 터미널 3: G2 도착·정지 후 WAIT_UNLOAD에서 유지
./scripts/smartcart_sim.sh delivery2
# 터미널 4: 운영자가 수령 확인. 이 터미널도 domain198/LOCALHOST로 격리
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=198 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_STATIC_PEERS ROS_DISCOVERY_SERVER
ros2 service call /smartcart/sim/confirm_receipt std_srvs/srv/Trigger '{}'
# 모의 관제가 H2 예약 → 로봇이 N12 → N08 → H2로 후진 주차
# 취소: 현재 세션의 대기/주행을 중단. 예약은 수동 복구 전까지 유지
ros2 service call /smartcart/sim/stop std_srvs/srv/Trigger '{}'
```

모의 관제는 현재 2초 이내 로봇 상태가 WAIT_UNLOAD일 때만 수령 확인을 받는다.
H1/H2 중 점유되지도 예약되지도 않은 칸을 선택하며, 둘 다 없으면 실패 응답을 내고 로봇은 계속 정지한다.
ROS `smartcart/sim/state`/`smartcart/sim/control`의 JSON은 이 시험 전용이다.
매 실행 새 session을 발급하고, 일치하는 session·명시적 receipt_confirmed=true·H1/H2·reservation_id·유한한 만료 시각을 검증한다.
같은 예약의 0.5초 heartbeat만 받으며 새 목적지나 늘어난 만료 시각으로 예약을 덮어쓰지 않는다.
예약은390초(수신 상한400초), heartbeat 2초 초과·만료·정지 요청 시 현재 Nav2 취소와 정지 명령 후 FAULT/CANCELED로 끝낸다.
승인 대기는 최대120초이며 시간 초과 시 정지 상태로 실패한다. 승인 후 모든 접근·라인 구간에서 기존 센서 신선도 검사도 유지한다.
도착 시 모의 관제는 배정 칸을 점유 상태로 유지한다. 이 fixture는 메모리만 사용하므로 재시작 후 예약 복구를 보장하지 않는다.
실행 중 재시작은 heartbeat 소실로 로봇을 중단시키며, 수동 복구 후 새 시험을 시작해야 한다.
다른 기체·MQTT 운영 인증·영속 점유 저장·실제 수령 확인·실물 IR/마커 전환은 별도 연결 및 현장 검증 대상이다.


2026-11-07 Gazebo 모의 관제 시험(각 복귀 1회, 중간 위치 재배치 없음):

| 조건 | 결과 |
|---|---|
| G2 도착 후 수령 승인 없음 | 약46초 정지 유지 |
| H1 점유, 수령 확인 | H2 배정 → N12/N08/H2 → 회전·후진 주차 arrived, 최종 속도0 |
| H1/H2 모두 점유 | 수령 확인 서비스가 No free bay로 거절, 정지 유지 |
| 미승인 상태에서 모의 점유 입력을 H2만 점유로 바꾸고 다시 수령 확인 | H1 배정 → N12/N08/N09/H1 → 회전·후진 주차 arrived, 최종 속도0 |

H2 최종 실제 모델 위치는 약(1.862,-1.709)m, yaw1.431rad, H1은 (1.541,-1.708)m, yaw1.497rad였다.
H1/H2 가로선과 바퀴 축은 IR 장착 거리만큼 다르며 충전 접점 성공을 뜻하지 않는다.
위 빈칸 변경은 승인 전 모의 관제를 재시작해 점유 입력을 바꾼 시험이다. 예약 영속 복구 시험이 아니다.
목표 도착 추정 오차는 두 G2 시험에서 약3.3cm/1.6cm였다. 실물·실제 운영 관제 메시지 연결은 미검증이다.


### 이동 장애물 배송·복귀 시험 · FFT1-120

```bash
# 담당 Nav2 구간만 시험: 라인 구간에는 물체를 생성하지 않음
./scripts/smartcart_sim.sh obstacles scenario:=nav-static height:=0.10 gui:=true
# 같은 위치에서 왕복 이동하는 물체: scenario:=nav-moving
# 아래 combined는 기존 라인 안전 정지 회귀 시험까지 포함한 별도 장면
# 실물 없이 독립 시뮬레이션. Nav2 구간의 상자는 6×6×12.5cm로 왕복 이동
./scripts/smartcart_sim.sh obstacles bay:=H1 gui:=true
# 라이다보다 낮은 10cm 상자로 H2를 시험하려면 bay:=H2 height:=0.10
# 별도 터미널: 반대편 칸 점유 → delivery2 → confirm_receipt는 위 모의 관제 절차와 같음
./scripts/smartcart_sim.sh control --occupied H2
./scripts/smartcart_sim.sh delivery2
```

기본 `scenario:=combined`는 주황 상자 2개를 생성한다.
`nav-static`은 공용공간 `(0.95,-0.55)`m에 고정 상자 하나만 생성하며, `nav-moving`은 같은 상자만 왕복 이동한다.
라인 구간에 장애물이 없는 배송 조건은 이 두 Nav2 전용 장면으로 시험한다.
공용공간 이동 상자는 x=0.95m에서 y=-0.85..-0.25m를 0.04m/s로 왕복한다.
라인 상자는 선택 칸 앞(y=-1.48m)에 있는 6×6×10cm 물체다. 해당 분기 근처에서 Collision Monitor 정지를 관측한 뒤
시뮬레이션 시간3초 정지하고, 0.05m/s로 옆으로30cm 이동한다. 이는 정지→통과→재개를 재현하는 시험 동작이며
관제의 로봇 목표·속도에는 접근하지 않는다. 상태 수신이1초 이상 끊기면 상자 이동도 멈춘다.
`stationary_line:=true`는 상자를 계속 두어 라인 차단 실패를 시험한다.

Nav2는 기존 라이다와 낮은 모의 거리 스캔을 장애물 지도에 반영해 재계획하고, 마지막 Collision Monitor가 감속·정지한다.
`inf_is_valid`로 생성되는 최대 거리의 지우기용 점을 장애물로 표시하지 않도록, 표시 최대0.94m와 광선 지우기0.95m를 구분한다.
초음파의 acoustic 반사·흡수·재질 특성은 구현하지 않았다. 실물 초음파 Range를 이 LaserScan 경로에 그대로 연결하면 안 된다.
이전 RangeSensorLayer의 초기 관측 지연 후 current 상태 복구 문제는 실제 Nav2 controller 회귀 시험으로 재현·교체 검증했다.
참고: [Nav2 Jazzy RangeSensorLayer 구현](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_costmap_2d/plugins/range_sensor_layer.cpp).

라인 구간은 탈선 우회하지 않는다. 감지된 장애물 때문에 정지한 동안 이동·회전 odom이 진행되지 않으면 기다리며,
12초(시뮬레이션 시간) 이상 계속 막히면 `line_motion_blocked` 오류로 정지하고 복구 지시를 기다린다.
그 전에 통로가 열리면 기존 라인 명령이 다시 통과한다. 센서 신선도0.5초·예약 heartbeat2초 기준은 유지한다.
장애물 대기·벽 앞 감속 회전 시간을 포함하도록 이 모의 복귀 CLI의 Nav2 목표 제한은180초로 둔다.
복귀 예약390초는 Nav2 180초+정렬40초+라인150초의 상한에 여유를 둔 값이다. heartbeat 소실 정지는 별도로 유지한다.

각 상자에는 Gazebo Contact 센서를 붙이고 실제 모델 pose와 함께 기록한다. 로봇의 위치/거리 평가에는 wheel odom+spawn 추정을 쓰지 않는다.
접촉 토픽은 Gazebo가 실제 발행하는 scoped sensor 경로를 ROS `sim/obstacles/{name}/contacts`로 remap한다.
출발·도착 테스트와 마찬가지로 이 장면은 domain198/LOCALHOST 전용이며 운영 MQTT·실물 설정을 변경하지 않는다.

2026-11-08 Gazebo 결과(각1회):

| 장면 | 확인 결과 |
|---|---|
| Nav2 전용 고정 상자6×6×10cm, 라인 물체 없음 | 공용공간 입구 출발→상자 우회→G2→수령 확인→N12→H1 주차 성공, 최종 정지, 로봇 접촉 이벤트0 |
| Nav2 전용 이동 상자6×6×10cm, 라인 물체 없음 | 공용공간 입구 출발→상자 회피→G2→수령 확인→N12→H2 주차 성공, 최종 정지, 로봇 접촉 이벤트0 |
| 6×6×12.5cm 이동 상자 + H1 앞10cm 상자 | G2→배정 H1 회전·후진 주차 성공, 최종 정지, 로봇 접촉 이벤트0 |
| 두 상자 모두 높이10cm + H2 배정 | G2→라인에서 실제 정지→상자 이동 후 재개→H2 주차 성공, 최종 정지, 로봇 접촉 이벤트0 |
| 별도 N12 출발 라인 fixture에서 H1 앞 상자를 고정 | 무동작12초 후 line_motion_blocked/FAULT, 최종 정지, 임의 탈선 없음 |

Nav2 전용 두 시험은 시작 전에만 모델과 AMCL을 공용공간 입구 `(2.55,-0.95,pi)`에 배치했다.
고정 시험 상자의 실제 좌표 변화는0m였고, 이동 시험 상자는0.04m/s로 왕복했다.
두 시험 모두 주행 중 로봇·상자를 재배치하지 않았다. 기본 launch의 spawn은 기존 home이므로
위 입구 왕복 시험과 출발 위치가 다르다. 고정 시험에서는 센서 관측 후 초기 경로 대신 상자 옆을 지나는 경로를 받는 것과 실제 모델 궤적을 함께 확인했다.
G2/복귀 인계점의 Nav2 보고 오차는 고정 약2.1/1.7cm, 이동 약2.7/1.3cm였다.
이는 AMCL 추정 기준이며, 실제 Gazebo 모델과 G2 목표의 거리는 각각 약7.5/6.2cm로 정밀 정차 보정이 남아 있다.
이 결과는 각 장면1회의 Gazebo 검증이며 실물·사람 인식·운영 MQTT 연동 완료를 뜻하지 않는다.

H1 시험 뒤 최대 거리 무반사 점 표시를 수정하고 H2를 최종 설정으로 시험했다.
H1의 라인 상자는 분기 진입 뒤3초 후 이동시키는 초기 fixture였고, H2는 실제 안전 정지를 관측한 뒤3초 후 이동시켰다.
두 성공 실행은 중간 위치 재배치·명령 재시도 없이 공용공간에서 출발했다. 영구 차단 시험만 별도로 N12에 배치한 실패 경로 검증이다.
Contact 센서 경로는 성공 시험 뒤 별도 접촉 probe를 상자와 겹쳐 실제 이벤트가 수신되는 것도 확인했다(로봇과의 접촉 시험이 아님).
각 결과는 반복 신뢰성·충전 접점·실물 검증을 대체하지 않는다.


## 실물 N12 → H1/H2 복귀 · FFT1-81

`real_return`은 관제와 분리한 감독 시험 명령이다. 게이트 등 공용공간에서 시작해
`return_entry` → `return_n12`까지 Nav2로 이동한 뒤, 실제 `ir_sensor/range`와 wheel odom을
사용하는 공용 `RouteExecutor`로 `N12 → N08 → H2`를 주행한다.
`--bay H1`이면 `N12 → N08 → N09 → H1`이며, 대기칸 예약·고객 수령 확인은 호출자/관제 책임이다.
이 실행부는 MQTT 계약이나 관제의 mode switch를 자동 변경하지 않는다.

```bash
# bringup은 별도로 실행. 기존 관제/라인 구동 실행부와 중복 실행하지 않는다.
export ROS_DOMAIN_ID=13
# 도메인은 예시다. 현장 기체에 배정한 값과 아래 --domain-id를 함께 맞춘다.
ros2 launch roport_smartcart real_return_stack.launch.py \
  robot:=ROBOT_NAME map:=/path/airport_map.yaml \
  layout_file:=/path/roport_airport.yaml params_file:=/path/nav2.yaml \
  max_linear:=0.03 max_angular:=0.15

# 기본은 설정 확인만 수행한다. 실제 이동은 현장 확인 후 --drive를 명시한다.
ros2 run roport_smartcart real_return \
  --domain-id "$ROS_DOMAIN_ID" \
  --robot ROBOT_NAME --calibration /path/robots.yaml --bay H2 \
  --map /path/airport_map.yaml --layout /path/roport_airport.yaml \
  --places /path/field-places.yaml
```

실기 장소 파일은 map YAML/이미지 해시와 `environment: real`이 맞아야 하며,
`return_entry`·`return_n12`는 현장 확인 후에만 `measured: true`로 등록한다.
`return_n12`는 복귀 방향(N08 방향)을 바라보고 실제 중앙 IR이 N12 테이프 위에 오는
바퀴 축 자세다(기본 IR 전방 거리 3cm). 출발 방향의 `handoff_n12`와 구별한다.
시뮬레이션 장소 파일의 `environment`만 변경해 사용하지 않는다.

전용 stack의 일반 Nav2 허용 오차는 기존 8cm를 유지한다. 실행부는 `return_n12` 목표에만
controller의 위치 허용 오차를 1cm로 일시 변경하고, 성공 후 실제 정지를 확인한 뒤 원래 값으로 복원한다.
실패·취소 때는 action 종료와 실제 정지 확인 후 정리 단계에서 복원한다. 파라미터 조회·변경·복원 실패는 오류로 종료한다.
2026-11-09 실물에서는 N12 실측 보정 후 1cm 설정으로 IR 획득과 H2 주차를 재시도 1회 성공했다.
반복 성공률과 모든 장소의 1cm 정차를 검증한 것은 아니므로 다른 목표에 이 설정을 일괄 적용하지 않는다.
Nav2 성공 결과·실제 연속 정지 0.5초·N12 자세(5cm/0.1rad 이내)와 실제 중앙 IR 정렬을
새 센서 샘플로 연속 0.5초 확인한 뒤에만 lease를 `none`에서 `line`으로 전환한다.
정렬 실패 시 10초 안에 정지 상태로 종료하며, 선을 찾겠다고 추정 좌표만으로 움직이지 않는다.
IR 0.3초/odom 0.5초 초과 지연, 센서 오류, 구동권 충돌, guard 재시작, 취소는 주행을 중단한다.
주차 완료는 기존 라인 실행부의 180도 회전·후진·실제 IR 가로선 감지와 실제 정지로 판정한다.
라인 속도는 기본 0.03m/s(`--line-speed 0.06` 선택 가능), 회전 상한은 0.15rad/s다.
N12·N08은 물류 Pinky와 공유하는 구간이다. 감독 시험에서는 다른 기체를 해당 구간 밖에 정지시키고,
관제 통합 시에는 구간 점유·예약을 별도로 연결해야 한다.
`trial_guard`만 최종 `cmd_vel`을 발행하며, lease 단절 1초/속도 단절 0.3초 감시를 유지한다.
`smartcart/return_state`에 단계·선택 칸·라인 상태 및 위치 정보 출처를 발행한다.

실행부와 모의 검증은 실기 주행 검증 완료를 뜻하지 않는다. 실제 N12 선 획득·전환·주차는
현장에서 별도로 확인하며, 앞선 Nav2 단독 H2 왕복 결과와 구분한다.

단계별 실기 검증에서 N12에 직접 놓은 경우에는 `--from-n12`를 추가한다. Nav2 이동만 생략하며,
실기 `return_n12` 장소 등록·지도 일치·실제 자세·연속 정지·새 IR 선 감지·구동권 확인은 동일하게 적용한다.

실측 N12 가로선에서 확인한 정지 자세와 IR 전방 거리로 복귀 정차점을 보정할 수 있다.
등록점은 명목 그래프와 5cm/0.1rad 이내인지 검사하고, 주행 때는 그 실측 등록점을 기준으로
기존 자세·연속 정지·실제 IR 획득 조건을 다시 확인한다.
