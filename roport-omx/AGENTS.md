# OMX

- 팀 코드는 `src/roport_omx/`, 원본 LeRobot은 `lerobot/`에서 구분한다. 원본 버전과 SHA는 `UPSTREAM.md`를 기준으로 확인한다.
- 코어 패키지와 LeRobot 학습/장비 의존성 환경을 섞지 않는다. Python·PyTorch·드라이버 호환성을 확인하고 lockfile을 임의로 업그레이드하지 않는다.
- 기존 검증: `uv sync --locked`, `uv run ruff check src`, `uv run ruff format --check src`, `uv build`. `ci/test.sh`가 있으면 함께 실행한다.
- 관절 상태·action의 feature 순서, 단위, gripper 정규화, 영상 크기·FPS·timestamp를 보존한다. ROS 메시지와 LeRobot 데이터 형식이 같다고 가정하지 않는다.
- 수동 장비 검사는 `scripts/check_hardware.py`가 있는 경우 그 범위를 확인해 사용한다. 모터 제어 프로세스와 시리얼 포트를 동시에 사용하지 않는다.
- 장비 미연결 상태에서는 토크·목표 위치·모터 ID·캘리브레이션을 쓰지 않는다. GPU 학습과 실장비 동작은 별도 검증으로 보고한다.
