# Pinky

- Ubuntu 24.04 / ROS 2 Jazzy를 기준으로 한다. 팀 패키지는 `src/roport_pinky/`, 원본 Pinky Pro는 `src/pinky_pro/`에서 구분한다.
- ROS 환경을 source한 뒤 `colcon build --packages-select roport_pinky`로 팀 패키지를 검사한다. `ci/test.sh`가 있으면 해당 CI 범위를 함께 확인한다.
- 토픽·메시지 타입·QoS·namespace·ROS_DOMAIN_ID·TF parent/child frame과 timestamp를 확인한다.
- 주행 속도·거리·회전 단위를 명시하고 실장비와 시뮬레이션 설정을 구분한다. 도킹·안전 정지의 판정 기준을 임의로 완화하지 않는다.
- bringup은 모터 초기화·토크 활성화를 수행할 수 있다. 수신 검사에서 bringup이나 cmd_vel 발행을 자동으로 실행하지 않는다.
- x86_64에서 ARM64 드라이버 코드가 제외될 수 있다. 빌드 성공을 실제 센서·구동기 검증으로 취급하지 않는다.
