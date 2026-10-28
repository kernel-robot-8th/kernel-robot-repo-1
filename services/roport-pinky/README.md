# roport-pinky

RO:PORT · GATE CREW — Pinky 주행·도킹·로봇 명령 실행

## 기술 스택

Ubuntu 24.04 / ROS 2 Jazzy / Python 3.12 / colcon / ament_python. Nav2 연동은 실장비 주행 방식 합의 후 추가합니다.

## 현재 범위

개발 시작용 구조와 CI를 제공합니다. 로봇 제어·업무 기능·모델 추론은 아직 구현하지 않았습니다.
CI 성공은 실로봇·GPU·통합 검증 완료를 의미하지 않습니다.
배포 환경 미정으로 CD와 자동 배포는 설정하지 않았습니다.

## 관련 문서

이 디렉토리는 단일 저장소의 구성요소입니다. 아래 명령은 이 디렉토리에서 실행합니다.

- [프로젝트 개요](https://github.com/kernel-robot-8th/kernel-robot-repo-1)
- [요구사항·설계 문서](../../docs/README.md)
- [Jira](https://fp-team1.atlassian.net/jira/projects/FFT1)

PR에 관련 Jira 키, 변경 내용, 검증 결과와 미검증 항목을 기록합니다.
실로봇 적용 전 장비·설정·캘리브레이션, 안전 정지·복구 절차를 확인합니다.

## 개발 방법

Ubuntu 24.04에 ROS 2 Jazzy와 colcon을 설치한 뒤 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select roport_pinky
source install/setup.bash
python3 -c "import roport_pinky; import rclpy"
```

팀 패키지에는 아직 실행 노드·모터 명령이 없습니다. 아래 원본 Pinky Pro에는 실장비 실행 코드가 포함돼 있습니다.

## Pinky Pro 원본

[src/pinky_pro/](src/pinky_pro/)에 공식 소스를 포함했습니다. 커밋과 라이선스는 [src/UPSTREAM.md](src/UPSTREAM.md)에 기록합니다. 원본은 Ubuntu 24.04 / ROS 2 Jazzy를 안내합니다.

원본 전체를 빌드하려면 의존성을 준비한 ROS 환경에서 실행합니다. 설치 명령은 시스템 패키지를 추가하며, 빌드 자체는 모터를 구동하지 않습니다.

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

기본 CI 실행: `bash ci/test.sh` — 팀 패키지·원본 인터페이스·description 빌드, URDF 전개와 수신 검사 로직을 검증합니다.

실로봇 bringup이 이미 실행 중인 상태에서 같은 `ROS_DOMAIN_ID`로 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/check_hardware.py --namespace / --output /tmp/pinky-check.json
```

`odom`·`joint_states`·`scan`에서 timestamp가 증가하는 메시지를 각각 3개 수신합니다. 명령을 발행하거나 bringup을 실행하지 않습니다. 실패 시 exit code 1과 JSON 결과를 남깁니다. 수신 성공만으로 실로봇/시뮬레이터 구별이나 동작 안전성을 판정하지 않습니다. ARM64 드라이버·전체 시뮬레이션·실주행은 별도 검증이 필요합니다.

## 테스트 웹 콘솔

[src/pinky_test_console/](src/pinky_test_console/README.md)은 SLAM 지도 제작과 Nav2·카메라 실행 시 메모리 확인을 위한 브라우저 콘솔입니다.
원본 Pinky Pro 웹 서버(Apache-2.0)를 바탕으로 웹 조종(0.10 m/s·0.6 rad/s 서버 제한, 0.3 s deadman, 정지 버튼)과 상태 패널을 더했습니다.
로그인 없이 로봇을 움직이므로 강의실 Wi-Fi 안에서만 사용합니다. 빌드는 `colcon build --packages-up-to pinky_test_console`입니다.
