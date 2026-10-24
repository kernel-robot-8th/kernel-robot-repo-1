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
- [요구사항·설계 문서](../docs/README.md)
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

현재 실행 노드·모터 명령은 없습니다. CI는 ROS 패키지 빌드와 설치 후 import만 검사합니다.

## Pinky Pro 원본

[src/pinky_pro/](src/pinky_pro/)에 공식 소스를 포함했습니다. 커밋과 라이선스는 [src/UPSTREAM.md](src/UPSTREAM.md)에 기록합니다. 원본은 Ubuntu 24.04 / ROS 2 Jazzy를 안내합니다.

원본 전체를 빌드하려면 의존성을 준비한 ROS 환경에서 실행합니다. 설치 명령은 시스템 패키지를 추가하며, 빌드 자체는 모터를 구동하지 않습니다.

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

CI는 팀 패키지 빌드와 원본 Python 구문 검사만 수행합니다. Pinky Pro 전체 빌드·시뮬레이션·실로봇 동작은 아직 검증하지 않았습니다. 원본에 포함된 장치·캘리브레이션·네트워크 설정은 각 장비에 맞게 확인해야 합니다.
