# roport-pinky

RO:PORT · GATE CREW — Pinky 주행·도킹·로봇 명령 실행

## 기술 스택

Ubuntu 24.04 / ROS 2 Jazzy / Python 3.12 / colcon / ament_python. Nav2 연동은 실장비 주행 방식 합의 후 추가합니다.

## 현재 범위

개발 시작용 구조와 CI를 제공합니다. 로봇 제어·업무 기능·모델 추론은 아직 구현하지 않았습니다.
CI 성공은 실로봇·GPU·통합 검증 완료를 의미하지 않습니다.
배포 환경 미정으로 CD와 자동 배포는 설정하지 않았습니다.

## 관련 문서

- [프로젝트 개요](https://github.com/kernel-robot-8th/kernel-robot-repo-1)
- [요구사항·설계 문서](https://github.com/kernel-robot-8th/kernel-robot-repo-1/blob/main/docs/README.md)
- [Jira](https://fp-team1.atlassian.net/jira/projects/FFT1)

PR에 관련 Jira 키, 변경 내용, 검증 결과와 미검증 항목을 기록합니다.
실로봇 적용 전 장비·설정·캘리브레이션, 안전 정지·복구 절차를 확인합니다.

## 개발 방법

Ubuntu 24.04에 ROS 2 Jazzy와 colcon을 설치한 뒤 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
python3 -c "import roport_pinky; import rclpy"
```

현재 실행 노드·모터 명령은 없습니다. CI는 ROS 패키지 빌드와 설치 후 import만 검사합니다.
