# OMX · LeRobot 버전 및 실행 환경

2026-10-24 정적 소스 확인 기준입니다. 실장비 연결·모터 구동·GPU 학습은 이 문서 작성 중 수행하지 않았습니다.

## 결론

기존 강의 재현·데이터·체크포인트 확인에는 LeRobot v0.4.4를 우선 기준으로 삼습니다.
다만 실제 실습 PC에 설치한 저장소·commit과 GPU가 아직 확인되지 않아 실장비 기준 버전 확정은 보류합니다.
신규 개발 후보인 0.6.2 계열은 별도 환경에서 검증하며 기존 환경과 섞지 않습니다.
`roport-omx`의 현재 Python 3.12 코어 CI는 LeRobot 호환성 검증이 아닙니다.

## A. 강의 재현 기준

- 공식 소스: Hugging Face LeRobot v0.4.4, commit `8fff0fde7c79f23a93d845d1a50e985de01f8b8a`.
- 태그의 Python 요구사항: `>=3.10`. 강의 재현 환경은 Python 3.10을 기준으로 별도 구성합니다. OS의 기본 Python은 교체하지 않습니다.
- OMX 모터 경로: `DynamixelMotorsBus`, `dynamixel-sdk>=3.7.31,<3.9.0` extra.
- torch 제약: `>=2.2.1,<2.11.0`; torchvision: `>=0.21.0,<0.26.0`.
- `draccus==0.10.0`, `av>=15,<16`, `opencv-python-headless>=4.9,<4.13`.
- PyTorch 2.7.1 + torchvision 0.22.1은 공식 배포 짝이며 이 태그의 선언 범위를 만족하는 초기 검토 후보입니다. 기존 체크포인트·GPU에 맞는 조합인지 별도 확인해야 합니다.
- CUDA wheel 변형, TorchCodec·FFmpeg 조합은 GPU·OS·decoder 확인 후 함께 고정합니다. torch·torchvision·TorchCodec을 각각 최신으로 설치하지 않습니다.
- 위 범위만으로 모든 의존성의 상호 호환성이 증명되지는 않습니다. 별도 잠금 파일 생성, 실제 import·영상 decode·학습 1 step이 필요합니다.

## B. 신규 개발 후보

- 로컬 분석본: `/home/seunghoon/Desktop/projects/lerobot`.
- 소스 버전 0.6.2, commit `c1789b1306ca451cdc61c4576c3eba691bd9b974`. 최신 main이라는 뜻은 아닙니다.
- Python `>=3.12`; torch `>=2.7,<2.12`; torchvision `>=0.22,<0.27`.
- NumPy `>=2.0,<2.3`, OpenCV headless `>=4.9,<4.14`, draccus `>=0.11.6,<0.12`.
- OMX는 `SerialMotorsBus`와 serial-motors extra를 사용합니다. v0.4.4의 DynamixelMotorsBus 경로와 다릅니다.
- 로컬 lock에서 확인한 조합은 torch 2.11.0, torchvision 0.26.0, TorchCodec 0.11.1, NumPy 2.2.6입니다. 이 lock의 존재는 팀 GPU에서 실행 성공을 의미하지 않습니다.
- 수집은 `lerobot-record`, 정책 실행은 `lerobot-rollout` 경로를 기준으로 확인합니다. 구버전의 `record --policy.path` 예제를 그대로 적용하지 않습니다.
- 채택 조건: 모터 버스 mock/실장비 확인, 기존 데이터 읽기·관측 비교·모델 로딩·행동 비교, 기존 버전 대비 성공률·지연 확인.

## C. 프로세스와 하드웨어 소유권

- ROS Bridge: ROS 2 Jazzy/Python 3.12 환경. 관제 명령을 전달하고 상태를 수집합니다.
- OMX Executor: 선택한 LeRobot 환경. USB/TTL 버스 접근과 명령 검증·정지·실행 이력을 소유합니다.
- 학습 프로세스: 데이터만 읽고 GPU를 사용합니다. 학습 시작이 모터 연결을 요구하는 구조로 만들지 않습니다.
- 추론 프로세스: 초기에는 OMX 제어 호스트와 가까운 환경을 우선 검증합니다. AI 서버 원격화는 제어 주기·관측 지연·행동 chunk 버퍼·통신 단절 테스트 후 결정합니다.
- 같은 팔을 LeRobot과 ROS/MoveIt이 동시에 제어하지 않습니다. 선택한 실행 경로 하나가 하드웨어를 소유하도록 합니다.
- 양팔 협동은 각 팔 ID·역할·시간 동기화·공유 작업 공간·정지 범위를 정의해야 합니다. 독립 정책을 동시에 시작하는 것으로 동기화를 보장하지 않습니다.

## D. 관절·단위·설정

- 확인한 드라이버의 모터 ID 가정은 11~16이며 관절 순서는 shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper입니다. 보유 장비에 그대로 적용하지 않고 읽기 확인합니다.
- 팔 5개 관절의 기본 값은 정규화 범위 -100~100이고 `use_degrees=True`이면 degree입니다. gripper는 0~100 정규화 값입니다.
- `.pos`를 ROS의 rad 또는 Cartesian pose로 해석하지 않습니다. ROS 연동 시 관절별 단위·방향·영점·변환을 명시합니다.
- `max_relative_target=None`이면 해당 상대 목표 제한이 설정되지 않은 상태입니다. 양의 제한값을 장비 시험으로 정하되 이 값만으로 충돌·힘·속도 안전을 보장하지 않습니다.
- 공장 사전 설정 장비도 robot ID, motor ID/model, 포트, calibration 파일과 장비 값이 일치하는지 확인합니다. 임의 재캘리브레이션·펌웨어 갱신은 하지 않습니다.
- 전원 차단·torque off 시 물품 낙하와 팔 처짐을 고려합니다. 통신 종료 시 무조건 torque off가 안전하다고 가정하지 않습니다.

## E. 데이터·체크포인트 계약

- 버전: LeRobot commit, dataset 형식 버전, dataset revision/hash, 모델 revision/hash.
- 관측: 카메라 이름·해상도·RGB/BGR·FPS·촬영 시각, 관절 이름·순서·단위·방향.
- 행동: 목표 명령과 실제 관절 상태 구분, 제어 주기, action chunk 길이·적용 주기, 제한·보간 방식.
- 캘리브레이션: 장비 식별자, calibration hash, 카메라 내부/외부 파라미터와 설치 상태.
- episode: 시작·종료·리셋, 성공·실패, 인간 개입, 관측·행동 동기화와 누락 프레임 처리.
- 학습/검증 분리: 같은 시연의 인접 프레임이 양쪽에 섞이지 않도록 episode 단위로 분리합니다.
- 정책 실행: 설정·processor·정규화 통계를 모델과 함께 로딩하고 episode 경계에서 정책 상태를 초기화합니다.
- 데이터 형식 변환은 원본 사본에 수행하고 읽기 결과·샘플 수·동기화·행동 단위를 비교합니다. rosbag 변환 성공만으로 호환성을 판단하지 않습니다.

## F. 확정 전 검증 순서

1. 실습 환경에서 저장소 URL·commit, Python·torch·torchvision·decoder·FFmpeg, GPU·driver, 장비 설정을 읽기 방식으로 수집합니다.
2. 기존 성공 환경을 보존하고 별도 환경에서 고정 의존성 설치·import·카메라 영상 decode를 검증합니다.
3. 장비 연결 없이 기존 dataset sample·checkpoint를 읽고 CPU 추론 출력 shape와 finite 여부를 확인합니다.
4. GPU에서 학습 1 step·모델 로딩·추론 지연/메모리를 측정합니다. 임의의 성공률·허용 지연을 확정값으로 쓰지 않습니다.
5. 사람이 작업 구역과 지지 상태·정지 수단·제한·복구 절차를 확인한 뒤 별도 승인된 실장비 시험을 진행합니다.
6. 단일 팔→상하차→양팔 협동 순으로 검증하고, 통과한 조합만 실제 배포 lock으로 승격합니다.

## 공식 근거

- [v0.4.4 의존성](https://github.com/huggingface/lerobot/blob/v0.4.4/pyproject.toml)
- [v0.4.4 OMX 드라이버](https://github.com/huggingface/lerobot/blob/v0.4.4/src/lerobot/robots/omx_follower/omx_follower.py)
- [0.6.2 분석 기준 소스](https://github.com/huggingface/lerobot/tree/c1789b1306ca451cdc61c4576c3eba691bd9b974)
- [ROBOTIS 설정 경로 구분](https://docs.robotis.com/docs/systems/omx/quick_start_guide/setup_guide/)
- [LeRobot OMX](https://huggingface.co/docs/lerobot/main/en/omx)
- [PyTorch 공식 버전 조합](https://pytorch.org/get-started/previous-versions/)
- [TorchCodec 호환성](https://github.com/meta-pytorch/torchcodec)

현재 ROBOTIS 예제와 최신 LeRobot CLI 예제가 서로 다른 버전을 설명할 수 있습니다.
명령을 가져올 때 반드시 실행 환경의 고정 commit과 비교합니다.
