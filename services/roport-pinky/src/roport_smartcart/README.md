# roport_smartcart

스마트카트(Pinky3, pinky_764e · PP_26091 · RAM 8 GB) 작업 패키지. 담당 강지원.
기능 명세는 Confluence「Tech Spec - 스마트 카트 게이트 안내 및 사용자 추종」(MFS 11960332, TS-3·TS-4),
공용 라인 주행 구조는「Tech Spec - 물류창고 Pinky」(MFS 12091401)의 SW 아키텍처 절을 따른다.

현재는 빈 골격이다. 실행 노드·모터 명령 없음.

## 역할 나눔

| 패키지 | 담당 | 내용 |
| --- | --- | --- |
| `roport_pinky_line` | 이민지 | **공용** IR 라인 주행: IR 판정·조향·정지(v0), 노드 감지·회전·경로(FFT1-117) |
| `roport_pinky` | 이민지 | 물류 Pinky 실행부(TS-1) |
| `roport_smartcart` (이 패키지) | 강지원 | 스마트카트 실행부, 라인→Nav2 전환, QR Reader, 고객 추종(P1), HMI |

라인 주행 코드는 복사하지 않고 `roport_pinky_line`을 그대로 실행한다. 기체 차이는 `roport_pinky_line/config/robots.yaml`의 `pinky_764e` 값으로만 둔다.

## 구성 제안 (구현 전)

```text
roport_smartcart/
  executor.py        # 상태 흐름: 대기 → 상차(P7) → 배송 → QR 대기 → 인도/안내/추종 → 복귀 (TS-3)
  mode_switch.py     # 라인 ↔ Nav2 전환: 입구 노드에서 라인 끄기 → 위치 재설정 → Nav2 시작
  qr_reader.py       # 카메라 + OpenCV QR, QR 대기 상태에서만 켬 (TS-3)
  marker_follower.py # 고객 마커(ID 35–49) 추종, P1 (TS-4)
  hmi.py             # LED·LCD·부저
config/places.yaml   # arrival, gate1–3, wait_p1–p3 지도 좌표
launch/smartcart.launch.py
```

## 공용 라인 주행 사용법 (인도장 P7 → 공용 내부공간 입구)

`roport_pinky_line`(PR #19) 병합 후 사용한다.

| 방향 | 토픽 | 형식 | 내용 |
| --- | --- | --- | --- |
| 실행부 → 라인 | `line_follower/enable` | `std_msgs/Bool` | true 출발, false 정지 |
| 라인 → 실행부 | `line_follower/state` | `std_msgs/String` (JSON) | `state`: idle·follow·line_lost·node·no_ir |
| 라인 → 구동 | `cmd_vel` | `geometry_msgs/Twist` | 켜져 있을 때만 발행 |

```bash
ros2 launch roport_pinky_line line_follow.launch.py robot:=pinky_764e
```

규칙:

1. 주행 소스(라인·Nav2·고객 추종)는 실행부가 한 번에 하나만 켠다. 다른 소스를 켜기 전에 앞 소스를 끄고 정지를 확인한다.
2. 라인 추종이 `node`로 서면 실행부가 다음 동작을 정한다. 입구 노드에서는 라인을 끄고 Nav2로 넘긴다.
3. `line_lost`·`no_ir`는 정지 상태다. 실행부가 관제에 보고한다.

## 빌드

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select roport_smartcart
```
