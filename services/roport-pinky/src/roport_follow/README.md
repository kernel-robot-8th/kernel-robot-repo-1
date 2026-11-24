# roport_follow

스마트카트 고객 카드 추종 중 인형 우회(Jira FFT1-185). 노트북이 AI 추론(YOLO·ArUco·바닥 거리)을 맡고 Pinky는 카메라·주행을 맡는 한 세트 구성이다. 로봇 쪽 추종 드라이버는 `roport_pinky`(`trial_follow`)에 있다. 기체별로 어떤 패키지가 스마트카트로 쓰이는지는 [기체별 실행 패키지](../../docs/smartcart-packages.md)를 본다.

지금 있는 것: 0단계 도구(FFT1-186), 인형 장애물 보내기·인형 기억(FFT1-187), 고객 카드 추종 대상 보내기(FFT1-188), 움직이는 인형 예상 자리(FFT1-190), 시험용 로봇 명령(`demo_runner`).

실로봇(d660, 2026-11-20~21, 37회): 멈춘 어린이 인형(6 cm, 카메라로만 보임)을 돌아 고객 카드를 따라가는 주행을 확인했다(인형 사이 25 cm 이상·고객 동선에서 30 cm 이상 떨어진 배치). 인형이 촘촘하면 돌아가는 동안 고객이 카메라 시야(좌우 약 ±24°) 밖으로 나가 놓침으로 끝난다. 움직이는 인형 앞 일시정지는 아직 없다. 아래 값은 시험값이며 승인된 기준값이 아니다.

## 정한 값 (2026-11-16)

| 항목 | 값 |
| --- | --- |
| 고객 카드 | ArUco `DICT_4X4_50` ID 35~49(팀 마커 표의 스마트카트 고객 칸), 지금 쓰는 카드 = 35·36(스마트카트 2대), 검은 사각형 30 mm(11-18 인쇄 시험), 인형 등에 붙임 |
| 고객 구분 | 카드를 단 인형 = 고객(장애물에서 뺌), 그 밖의 인형 = 피할 존재 — 카드 추종과 함께 구현 예정 |
| 사람 | 시연 중 사람이 들어오지 않아 사람 정지는 넣지 않음(11-19). 모델의 `human` 은 장애물로 안 씀 |
| 인형 위치 | 라이다(바닥 위 약 12.5 cm)에 인형이 안 잡혀 YOLO 상자 아래 끝 + 바닥 기하로 구함. 약 1 m 안에서만 믿을 만함 |

## 0단계 도구 (노트북)

로봇은 카메라만 쓴다(SSH 로 `robot_cam_stream.py` 를 보내 Picamera2 영상을 MPJPEG 로 받음). bringup·모터 명령은 하지 않는다. 다른 프로그램(테스트 콘솔 `mode:=marker` 등)이 카메라를 쓰고 있으면 열리지 않는다.

```bash
PY=~/dev_ws/roport/venvs/follow/bin/python      # ultralytics(CPU)·OpenCV 가상환경, 저장소 밖
cd ~/dev_ws/roport/kernel-robot-repo-1/services/roport-pinky/src/roport_follow
PYTHONPATH=. $PY -m roport_follow.tool view --robot <로봇 IP> --lens-height-cm 6.3
PYTHONPATH=. $PY -m roport_follow.tool web --robot <로봇 IP> --lens-height-cm 6.3   # 브라우저 http://127.0.0.1:8090/
PYTHONPATH=. $PY -m roport_follow.tool calib-intrinsics --images ~/roport_follow/calib/6422
PYTHONPATH=. $PY -m roport_follow.tool calib-floor --image <바닥 체스보드 사진>
PYTHONPATH=. $PY -m roport_follow.tool latency --robot <로봇 IP>
```

- `view`: 실시간 화면에 바닥 거리 선(25~200 cm, 그 거리에서 화면 위 끝 높이 「top」), YOLO(`y`, 기본 인형 모델 adult·child·human), 고객 카드(`a`, 거리·한 변 화소), 체스보드(`b`)를 겹쳐 보인다. 클릭하면 그 점의 바닥 거리를 알려 준다.
  - 카메라 자세 맞춤 두 가지: (1) 줄자 맞춤 — 거리(cm)를 숫자로 치고 줄자 표시를 클릭, 여러 점 뒤 `h`. 렌즈 높이를 `--lens-height-cm` 로 주면 높이를 고정한다. (2) 바닥 체스보드 — `f`. 보드 **긴 변을 로봇 앞 방향**으로, 검은 칸 시작이 렌즈 아래 바닥에서 약 8~12 cm가 되게 둔다(합성 영상 시험에서 가로로 두거나 30 cm보다 멀면 모서리를 못 찾음). 결과는 `w` 로 `~/roport_follow/camera_6422.json` 에 저장.
  - 측정 기록(시험 0): 줄자 거리(cm)를 치고 Enter → YOLO 상자 바닥 거리·키 거리(`--doll-height-cm`)·카드 거리를 `~/roport_follow/measure/*.csv` 에 남긴다.
  - `s`: 원본 사진 저장(`--prefix`, 친 거리 `_dNNN` 이 이름에 붙음). `r`: 연속 저장. `v`: 원본 영상 녹화(겹쳐 그리기 전 영상, `~/roport_follow/video/<prefix>_<시각>_raw.mp4`, 다시 누르거나 도구를 끄면 닫힘). 사람이 찍힌 사진·영상은 지우고 커밋하지 않는다.
- `web`: `view`와 같은 처리·키를 브라우저 화면으로 연다(실시간 영상, 체스보드 「검출됨/못 찾음」, 렌즈 보정 사진 장수, 바닥 자세·줄자 맞춤 판정, 측정 기록 버튼). 기본은 노트북 안(127.0.0.1)에서만 열린다. `--bind 0.0.0.0`은 인증이 없어 같은 망 누구나 버튼을 누를 수 있으니 쓰지 않는다. 체스보드 검출을 켜면 초당 약 5장으로 느려진다.
- `calib-intrinsics`: `c` 로 모은 체스보드 사진(15~20장, 각도·위치를 바꿔 가며) → 렌즈 초점·왜곡.
- `latency`: 노트북 화면의 시계 마커(`DICT_4X4_250`, 40 ms마다 바뀜)를 로봇 카메라로 찍어 화면 → 노트북 수신 지연을 잰다(±40 ms, 시계 맞춤 필요 없음).
- YOLO 기본 모델 = 인형 모델 `~/dev_ws/roport/models/doll_ft_20261119/best.pt`(adult·child·human, 저장소 밖, 가중치 커밋 금지). COCO 는 `--model ~/dev_ws/roport/models/yolo11n.pt`(없으면 처음에 내려받음). 노트북 CPU(i7-1165G7)에서 640×480 장당 약 40 ms.

## 인형 장애물 → Nav2 (FFT1-187, 시험 1)

노트북이 YOLO 인형 상자(adult·child, 신뢰도 0.35 이상, 1.2 m 안)를 바닥 점으로 바꿔 `/doll_obstacles`(PointCloud2, `base_footprint`)로 보낸다. 로봇 Nav2 의 지역·전역 비용 지도에는 `doll_layer`(ObstacleLayer)를 더한다. 이 도구는 모터 명령·Nav2 목표를 보내지 않는다.

로봇 Nav2 를 켜는 방법은 두 가지다. 카드 추종 실로봇 시험(11-20~21)은 (가)로 했다.
- (가) `roport_pinky` `trial.launch.py` + 인형 층이 들어간 전체 설정 `params_file:=<share/roport_pinky/config/nav2_follow_real.yaml>`(아래 「고객 카드 추종」 절).
- (나) `roport_smartcart` `navigation.launch.py` 에 인형 층만 더한 덮어쓰기 파일을 `overrides_file:=` 로 넘김(아래 명령, 11-19 멈춘 인형 우회 시험).

```bash
# 1) 덮어쓰기 파일 만들기(노트북, 저장소 원본을 읽음) → 로봇으로 복사
cd ~/dev_ws/roport/kernel-robot-repo-1/services/roport-pinky/src
PYTHONPATH=roport_follow python3 -m roport_follow.nav2_doll \
  --base-params pinky_pro/pinky_navigation/params/nav2_params.yaml \
  --overrides roport_smartcart/config/nav2_overrides.yaml --out ~/roport_follow/nav2_overrides_doll.yaml
# 2) 로봇: navigation.launch.py mode:=real ... base_params_file:=<main nav2_params 사본> overrides_file:=<로봇에 복사한 경로>
#    (Nav2 실행은 사람이 함. roport_smartcart 는 roport_pinky.navigation_policy 도 필요 → 두 패키지를 같이 빌드)
# 3) 노트북: 로봇과 같은 ROS_DOMAIN_ID(d660 = 14), 같은 Wi-Fi
source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=14
cd roport_follow && PYTHONPATH=".:$PYTHONPATH" $PY -m roport_follow.tool web --robot <IP> --name d660 \
  --camera ~/roport_follow/camera_d660.json --publish-obstacles     # http://127.0.0.1:8090/
```

`--publish-obstacles` 이면 웹 화면이 **시험 화면**으로 바뀐다(보정 카드는 접힘).
- 위쪽: 발행 상태(보내는 중 n Hz / 1 s 넘게 못 보내면 빨간 「멈춤」 / 일시 멈춤), 로봇 → 노트북 지연, 밀린 영상 수, ROS_DOMAIN_ID.
- 가운데: 위에서 본 지도(로봇·지우는 시야·표시 한계·보낸 인형 점·기억한 인형)와 인형 표(앞·왼쪽 cm, 폭). 「장애물 보내기 잠시 멈춤」= 끊김 시험(도구를 끄지 않고 보내기만 멈춤).
- 오른쪽: 시험 순서 1~6(클릭해서 고름, 다시 누르면 완료 표시), 메모 + 「기록 남기기」 → `~/roport_follow/measure/<name>_<시각>_test1.csv`(시각·단계·메모·발행 상태·인형 목록). 설정 확인: 카메라 파일(6422 파일이면 빨강), 표시 거리와 `--nav2-file`(기본 `~/roport_follow/nav2_overrides_doll.yaml`) 일치 여부.

- 거리는 모두 바퀴 축(`base_footprint`) 기준. 웹 도구 `--max-range-cm` 과 `nav2_doll --max-range-cm` 은 **반드시 같게**(다르면 지움 점이 로봇 앞을 따라다니는 벽으로 칠해짐).
- 점: 상자 아래 끝을 1 cm 간격으로 바닥에 투영 + 안쪽 두께(상자 폭, 3~8 cm), 높이 5 cm. 발이 잘린 상자(화면 아래 끝)는 로봇 쪽으로 2 cm 더 칠한다(11-22: 5 cm 면 코앞 인형 표시가 로봇 회전 반경 안까지 와서 꼼짝 못 함).
- 지움: 시야 안 1.25 m 부채꼴 점을 매번 같이 보낸다. 광선이 그 사이 칸을 비워 치운 인형이 사라진다. 광선 끝 간격 0.8 cm(2° 간격이면 1 cm 칸이 남는 것을 costmap 노드 시험에서 확인).
- 인형 기억(기본 켬, `--no-doll-memory` 로 끔): 어린이 인형(6 cm)은 라이다에 안 보이고, 카메라는 바퀴 축 약 13 cm 안을 못 본다. 두 번 이상 본 인형은 노트북이 odom 좌표로 기억해 영상마다 다시 보내고(`doll_memory.py`), 시야 밖 방향도 지우는 둘레 점(`clear_ring`, 끝 간격 2 cm)을 함께 보낸다. 로봇 Nav2 인형 층은 노트북이 이번에 보낸 것만 비춘다: `obstacle_min_range`·`raytrace_min_range` 0, `observation_persistence` 0(`nav2_doll` 기본값). 그 영상 시각의 `/odom` 자세가 없으면 기억을 갱신하지 않고 둘레 지움 점도 보내지 않는다(보이는 것만 보냄). 기억을 잊는 때: 카메라가 볼 수 있는 자리인데 3장 연속 안 보임 / adult 가 시야 밖 3 s / 로봇이 1.5 m 넘게 멀어짐 / child 가 시야 밖 60 s / 고객으로 확인한 검출과 같은 인형(같은 종류·10 cm 안) / 로봇이 라인 모드일 때(손으로 옮긴 뒤 옛 기억 방지). `/odom` 이 없으면 기억하지 못하고 보이는 것만 보낸다(화면 `memory.pose_ok`). 실로봇(d660, 11-21): 인형이 카메라 사각에 들어와도 비용 지도에 남아 돌아가는 것을 확인했다.
- 인형 기억을 끈 예전 방식(`--no-doll-memory` + `nav2_doll --keep-near-cm 15 --persistence-s 1`)에서는 로봇 축 15 cm 안(`raytrace_min_range`)을 지우지 않는다. 렌즈 앞 약 10 cm 는 카메라 사각이라 가까이 다가간 인형 표시를 남기기 위해서다. 이 방식은 Nav2 가 비용 지도를 비우면 그 표시가 사라져 인형을 쳤다(11-21) → 기본은 인형 기억이다. 화면 아래 모서리 쪽은 사각이라 시야 양쪽 6°는 어느 방식에서도 지우지 않는다(실제 보정 파일 d660·6422 로 「못 보는 칸을 지우지 않음」 시험).
- 시각: 프레임의 로봇 시각(`X-Robot-Time`)으로 보낸다(노트북 시계와 무관). 안 보내는 영상: 노트북이 받은 뒤 0.5 s 넘게 지난 것, 로봇 → 노트북 지연이 지금까지 최솟값보다 0.5 s 넘게 큰 것(화면 `lag`), 로봇 영상인데 로봇 시각이 없는 것.
- 끊김: 지역 비용 지도 `expected_update_rate`(`nav2_doll --expected-s` 기본 1 s, 추종 시험 설정 `nav2_follow_real.yaml` 은 2 s). 노트북이 그 시간 넘게 안 보내면 비용 지도가 「최신 아님」이 되어 Nav2 controller 가 멈춘다(11-19 d660: 발행이 끊기면 약 2.5 s 안에 controller 중단). 카드 추종에서는 `roport_pinky` 드라이버가 status 2 s 미수신으로도 추종을 끝낸다.
  - 한계: Nav2 복구가 비용 지도를 비우면(ClearEntireCostmap) 그 층이 잠깐 다시 「최신」이 된다. 추종 BT(`follow_target.xml`)는 비용 지도를 비우지 않는다.
  - 순서: **노트북 도구를 먼저** 켜고 점이 나가는 것(화면 `sent` 증가)을 본 뒤 Nav2 목표를 보낸다. 거꾸로면 목표가 바로 실패할 수 있다.
  - `mode:=sim` 에는 이 덮어쓰기를 쓰지 않는다(발행 쪽이 없어 늘 「최신 아님」).
- 덮어쓰기 파일은 만들 때의 `roport_smartcart` 값 복사본이다. `roport_smartcart` 의 `nav2_overrides.yaml` 이 바뀌면 다시 만든다.
- 전역 비용 지도에서 시야 밖에서 치운 인형 표시는 남는다(멈춘 인형 시험 1은 괜찮음, 움직이는 인형 FFT1-190 에서 다시 봄).
- 렌즈 위치 `--cam-x-cm`(기본 3.5, URDF 카메라 링크) — 실측하면 바꾼다. 카메라 값은 기체에 맞게 `--camera`(d660 = `camera_d660.json`, 기본값 6422 파일이면 경고).
- ROS 환경을 source 한 터미널에서만 켜진다(`rclpy` 는 venv 밖 `/opt/ros/jazzy`, venv 에 `pyyaml` 필요).

## 기하 근거

기본값은 10-31 실측(초점 약 540 px, 렌즈 높이 6.3 cm, 58 cm 앞 벽에서 12 cm까지 보임 → 숙임 약 18.35°, 추정)이다. 바닥이 평평하다고 보고 화소 ↔ 바닥 점을 바꾸며 1 px 오차는 거리 d에서 약 (d² + h²) ÷ (초점 × h)다(60 cm 약 1.1 cm, 100 cm 약 3 cm). 6422 기체 값은 0단계에서 다시 잰다.

## 테스트

```bash
python3 -m pytest -q -p no:cacheprovider src/roport_follow/test   # services/roport-pinky 에서, ci/test.sh 에 포함
```

바닥 기하 왕복·roll, 바닥 체스보드로 높이·숙임·roll 복원, 줄자 맞춤, MPJPEG 읽기, 카드 PnP 거리·범위 밖 ID 무시, 지연 계산, 원본 녹화, 인형 상자 → 비용 지도 점(클래스·신뢰도·거리 거름, 발 잘림, 지움 부채꼴 간격, 로봇 시각·밀린 영상), Nav2 덮어쓰기(plugins·smartcart 값 유지, `build_params` 통과)를 확인한다. 하드웨어·YOLO 없이 돈다.

## 고객 카드 추종·움직이는 인형·시험용 로봇 명령 (FFT1-188·190, 2026-11-21 시험값)

```bash
# 노트북(같은 ROS_DOMAIN_ID): 인형 장애물 + 움직이는 인형 예측 + 카드 35 추종 대상 + 시험용 로봇 명령 버튼
PYTHONPATH=".:$PYTHONPATH" $PY -m roport_follow.tool web --robot <IP> --name d660 \
  --camera ~/roport_follow/camera_d660.json --publish-obstacles --predict-moving \
  --follow-card 35 --demo --torch-threads 4
# 로봇(trial.launch 이 이미 떠 있을 때, 시험용 — 관제 MQTT 수신기 대신):
#   ros2 launch roport_pinky trial.launch.py ... params_file:=<share/roport_pinky/config/nav2_follow_real.yaml>
ros2 run roport_follow demo_runner --map <지도.yaml> --layout <레이아웃.yaml> --start-node N08
#   로봇을 N12 에 놓고 Nav2 모드로 바로 시작(①·② 건너뜀): --start-node N12 --start-mode nav2
```

- `--follow-card 35`: 카드 아래 바닥 점 → `/follow/target`(PointStamped, base_footprint), 상태 → `/follow/status`(JSON,
  searching·tracking·lost, `--follow-lost-s` 기본 1 s = 요구사항 SR-22. 실로봇 시연은 `--follow-lost-s 4` 를 명시하고
  로봇 명령에도 `lost_after_s: 4` 를 준다). 1.5 m 넘는 카드·튀는 검출은 버림.
  이번 영상에서 카드를 본 경우에만, 카드 둘레(adult 30 cm, child 15 cm)의 검출 중 고객 인형 하나(어른 인형 먼저, 가장 가까운 것)를
  장애물에서 뺀다. 고객 옆에 따로 선 인형과 카드가 가려진 동안의 검출은 장애물로 남는다.
  카드 모서리는 서브픽셀 보정(30 mm 카드는 20 px 안팎이라 1 px = 거리 5 %). 실제 정확도는 멈춘 카드 실측으로 확인할 것.
- `--predict-moving`: `/odom` 으로 로봇 움직임을 빼고 인형 속도(3~50 cm/s)를 구해 1 s 뒤까지 예상 자리도 장애물 점으로 보냄.
  odom 이 없으면 예측하지 않음(지금 동작 그대로).
- `--follow-doll-fallback`(**임시**, 기본 끔): 카드가 안 보이는 동안 마지막 고객 자리 근처(30~60 cm)의 어른 인형으로 최대 8 s 이어 추적.
  후보가 둘 이상이면 쓰지 않고, 카드 없이 추종을 시작하지 않는다(status `source: doll` 이면 로봇 드라이버가 새 추종 출발을 거절).
  팀 기준(추종 대상 = 카드)과 달라 시험용으로만 둔다.
- `--demo` + 로봇 `demo_runner`: 화면 「① N08 → N12 라인 출발 · ② Nav2 전환 · ③ 추종 시작 · ■ 정지」를 로봇 TrialDriver 에
  관제 trial-v1 과 같은 모양의 명령으로 넣음. 브라우저가 3 s 넘게 화면을 안 보면 살아 있음 신호가 끊기고 로봇이 멈춤
  (시험 탭을 앞에 둘 것). 관제에 추종 명령이 생기면 빼는 시험용 지름길이며, 최종 구조는 폰·패드 → 서비스 서버 → 관제 → 로봇.
- `tool web --boarding-alert`(기본 끔): `POST /api/boarding_alert {"on": true|false}` 를 받아 `smartcart/boarding_alert`(Bool, 2 Hz)를 내보낸다(`boarding_alert.py`). 패드를 띄운 노트북이 탑승 시간 알림을 로봇 LCD·LED·부저(`status_indicator smartcart:=true`)에 보이는 통로다. 주행 명령이 아니고, 60 s 뒤와 도구 종료 때 저절로 끈다. 로봇과 같은 ROS_DOMAIN_ID 여야 닿는다. 실로봇 미확인.
- `demo_runner` 의 `alert {"on": true|false}`: 탑승 시간 알림을 `smartcart/boarding_alert`(Bool, 2 Hz)로 내보낸다. 로봇 LCD·LED·부저는
  `roport_line_nav status_indicator`(smartcart:=true)가 맡는다(같은 LCD 를 쓰는 다른 표시 프로그램과 함께 켜지 않음). 주행 명령이 아니다. 화면(콘솔) 신호가 끊기면 알림도 끈다.
- `demo_runner` 의 Nav2 목표 도착 허용은 기본 0.12 m. `navigate` 에 `position_tolerance_m`(0.05~0.30)을 주면 그 값을 쓴다(ARR·Gate 처럼 목표가 인형·고객 둘레에 걸려 플래너가 0.3 m 안에서 경로를 끝낼 수 있는 목표만 넓힘).
  추종 명령에 `turn_first: true` 를 주면 지금 자리에서 180° 돈 뒤(`demo_TURN`, 같은 위치·반대 방향 Nav2 목표) 1.5 s 기다렸다가
  추종을 보낸다. ARR 에서 마커를 보고 선 카트가 뒤의 고객 쪽으로 돌아서는 용도다. 돌기만 하고 기다리려면 `turn` 명령(같은 `demo_TURN` 목표, 추종은 보내지 않음). 위치를 모르면 `turn_pose_unavailable`. 정지·새 명령·콘솔 끊김이면 기다리던 추종도 버린다(기다리는 동안 상태 `pending: follow_after_turn`).
- `demo_runner --fine-align`(기본 끔): N12 복귀 자세(`demo_RET_N12`)에서 Nav2 가 20 cm 뒤에 선 뒤 직선 정밀 접근(GateDock 2 cm/s,
  1.5 cm·0.08 rad 확인)을 한 번 더 한다(드라이버의 게이트 마커 정밀 접근 단계를 마커 대신 목표 좌표로 씀). Nav2 설정에 GateDock 제어기·
  `gate_dock_goal_checker` 가 있어야 하므로 `config/nav2_follow_real.yaml` 로 띄운 경우에만 켠다. 옵션이 없으면 기존 N12 복귀 그대로다.
- `--torch-threads 4`: YOLO CPU 스레드 수(노트북 8코어에서 기본값보다 빠름, 실로봇에서 비교할 것).
- 로봇 쪽 추종(`follow.requested`, Nav2 동적 따라가기 BT)은 `roport_pinky` README 의 「고객 카드 추종」 절.
- 주행 중에는 노트북·로봇에서 ROS 프로그램을 켜고 끄지 않는다(구동 감시 상태가 0.3 s 끊기면 로봇이 멈추고 잠김). 노트북 도구를 다시 켰으면 로봇 trial.launch → demo_runner 순서로 다시 켠다.
