# OMX 현장 운영 안내 (제어판 v3 · On-site 패널)

인도장 OMX 두 팔(OMX1 왼팔·OMX2 오른팔)로 관제 미션을 돌릴 때 쓰는 두 화면을 받고, 설치하고, 쓰는 방법입니다. 담당자가 없어도 팀원이 이 문서만 보고 따라 할 수 있게 씁니다. Host와 제어판이 맞물리는 API·버전 기준은 [제어판 연동 기준](control-panel.md)에 있습니다.

## 1. 무엇을 어디서 받나

| 화면 | 하는 일 | 코드 위치 | 받는 곳 |
| --- | --- | --- | --- |
| **OMX Control Panel v3** (제어판, `127.0.0.1:8865`) | 로봇 팔·카메라 연결, 녹화·학습·평가, armed 스위치, AI 추론(UDP) 연결. 관제 Host가 이 제어판에 평가를 요청합니다 | 별도 공개 저장소 [`tjkoobro-netizen/omx-control-panel`](https://github.com/tjkoobro-netizen/omx-control-panel) | 제어 PC: `~/omx-control-panel` |
| **OMX On-site Control Panel** (현장 패널, `127.0.0.1:8870`) | 관제 Host 시작·정지, armed, 관제 명령 현황, HOLD 해제, 카메라 확인, 가상 관제로 미션 시험 | 이 저장소 [`services/roport-omx/tools/onsite-panel/`](../tools/onsite-panel/) | Host PC: 이 저장소 clone(예: `~/roport-baseline/repo`) |
| 관제 Host 프로그램 | 관제(Fleet) MQTT 창구. 명령 접수·판정·보고, HOLD | 이 저장소 `services/roport-omx` (`scripts/cooperative_host.py`) | 위와 같은 clone |

- **Host PC**: 인도장 현장 PC입니다. OMX1·2와 카메라 USB가 연결돼 있습니다. 제어판 v3, Host, 현장 패널이 모두 이 PC에서 돕니다.
- **GPU 노트북**: 학습과 AI 추론 서버(UDP)를 맡습니다. 제어판 v3를 GPU 쪽에도 설치합니다(2-2).
- **이 저장소에 넣지 않는 것**: 브로커 비밀번호, CA 인증서, UDP 공유 키(`~/.config/roport/omx-udp.key`), 실제 카메라 사진. 담당자에게 따로 받습니다.

## 2. OMX Control Panel v3 (제어판)

### 2-1. 제어 PC(Host PC) 설치·업데이트

처음 설치할 때:

```bash
sudo apt install -y git
git clone https://github.com/tjkoobro-netizen/omx-control-panel.git ~/omx-control-panel
cd ~/omx-control-panel && bash install_control.sh
```

업데이트할 때(녹화·평가·미션이 돌고 있지 않을 때):

```bash
cd ~/omx-control-panel && git pull && bash install_control.sh
bash ~/il_ws/omx_control_panel_v3/omx_tools/omx-panel-server.sh --restart   # 켜져 있던 제어판 다시 시작
```

- 설치 위치는 `~/il_ws/omx_control_panel_v3/`이고, 바탕화면 아이콘은 **OMX Control Panel v3**입니다.
- 설치 중 묻는 것(Tailscale, GPU 노트북 계정, LeRobot 설치, USB 권한)은 제어판 저장소의 `INSTALL.md` 4절에 있습니다.
- `dialout` 권한이 새로 생겼다고 나오면 로그아웃 후 다시 로그인합니다.
- 버전별 변경은 제어판 저장소의 `CHANGELOG.md`에 있습니다. 화면 왼쪽 위에 지금 버전이 나옵니다.

### 2-2. GPU 노트북

- 제어 PC 설치 중 4/8 단계에서 GPU의 내 계정에 제어판을 원격으로 설치합니다.
- GPU 쪽만 다시 설치할 때는 제어판 저장소의 `install_gpu.sh`를 GPU의 내 계정에서 실행합니다. 계정 준비(관리자 `sudo`)는 `INSTALL.md` 3절에 있습니다.
- GPU 쪽 제어판이 AI 추론 UDP 서버(`omx_tools/infer_link`)를 띄웁니다. 이 서버 파일은 이 저장소의 `services/roport-ai-server/src/omx_udp`가 원본이고, 제어판 저장소에는 같은 코드의 사본이 있습니다. 그래서 서버 파일을 고치면 제어판 사본도 같은 커밋으로 맞춘 뒤 배포합니다.

### 2-3. 제어판 사용

1. 바탕화면 **OMX Control Panel v3**를 실행합니다. 처음에는 우클릭 → 실행 허용이 필요할 수 있습니다.
2. **로봇 구성**에서 왼팔·오른팔과 카메라 역할을 확인합니다.
   - 카메라 USB를 옮기면 자동으로 다시 찾습니다.
   - 화면 확인이 필요하다고 나오면 카메라 화면을 보고 저장합니다.
3. **평가** 탭
   - 화면에서 직접 평가할 때는 모델을 고르고, 로봇 옆 사람 확인을 체크한 뒤 시작합니다.
   - 관제 미션에서는 평가를 Host가 대신 시작합니다.
4. **armed**: 관제 명령으로 팔이 움직이려면 사람이 화면에서 켜야 합니다.
   - 제어판을 다시 켜면, 또는 중지·즉시 중단·비상 정지·오류가 나면 저절로 꺼집니다.
   - Host가 떠 있지 않은 채 armed를 켜면, 제어판이 `~/.config/roport/omx-host.json` 설정으로 Host를 직접 띄웁니다. 현장 패널을 쓸 때는 현장 패널에서 Host를 먼저 시작합니다.
5. **비상 정지 (제자리)**는 토크를 켠 채 그 자리에 멈춥니다. 같은 버튼 **[홈 위치로 이동]**을 누르면 천천히 돌아갑니다.
6. **손실 감시**(평가 탭)는 미션 중 카메라·통신·CPU 손실을 기록합니다. 기본으로 켜집니다.
7. 제어 PC에서는 **제어판 창을 닫으면, 돌고 있는 작업이 없을 때 제어판 서버가 스스로 꺼집니다**. 미션 중에는 창을 열어 두고, 다시 쓸 때는 아이콘으로 켭니다.

## 3. OMX On-site Control Panel (현장 패널)

### 3-1. 받기와 Host 환경

이 저장소를 Host PC에 받습니다. 경로는 `omx-host.json`의 `repo`와 맞춥니다. 예시 설정은 `~/roport-baseline/repo/services/roport-omx`입니다.

```bash
git clone https://github.com/kernel-robot-8th/kernel-robot-repo-1.git ~/roport-baseline/repo
cd ~/roport-baseline/repo/services/roport-omx
python3 -m pip install uv==0.13.5    # uv가 없을 때
uv sync --locked
```

업데이트할 때(진행 중인 명령이 없을 때):

```bash
cd ~/roport-baseline/repo && git pull
```

pull 뒤에는 현장 패널에서 **Host 정지 → Host 시작**을 눌러야 새 Host 코드로 돕니다. 현장 패널 서버도 바뀌었으면 터미널에서 `run.sh`를 다시 실행합니다.

### 3-2. 현장 설정 파일 (Host PC, 저장소 밖)

| 파일 | 내용 | 만드는 법 |
| --- | --- | --- |
| `~/.config/roport/omx-host.json` | Host 코드 위치(`repo`), 규약(`protocol` v2), 관제 ID(`control`), 브로커 설정 파일(`broker_env`), 경로표(`routes`), journal | [`config/omx-host.example.json`](../config/omx-host.example.json)을 복사해 경로를 맞춤 |
| `~/.config/roport/shelf-01-routes.json` | 칸·작업별 모델 경로, `model_version`·`calibration_version`(관제 설정과 같아야 함), 상자 번호 | [`config/shelf_routes.example.json`](../config/shelf_routes.example.json)과 [`config/cooperative_routes.example.json`](../config/cooperative_routes.example.json) 참고. 현장 값은 담당자에게 받음 |
| `broker_env` 파일(예: `~/roport-baseline/run/broker.env`) | `ROPORT_MQTT_HOST`, `_PORT`, `_TLS`, `_CA_FILE`, `_USERNAME`, `_PASSWORD` | 관제 담당에게 받음. **저장소에 넣지 않음** |

각 항목의 자세한 뜻은 [제어판 연동 기준 4절](control-panel.md#4-현장-설정-파일-host-pc)에 있습니다.

### 3-3. 실행

```bash
cd ~/roport-baseline/repo/services/roport-omx/tools/onsite-panel
bash install_desktop.sh    # 한 번만: 바탕화면 "OMX On-site Control Panel" 아이콘
```

아이콘을 누르면 서버가 켜지고 브라우저에 `http://127.0.0.1:8870`이 열립니다. 이 PC에서만 열립니다. 서버만 켤 때는 `./run.sh`를 씁니다.

### 3-4. 관제 미션 순서 (v2, 실제 관제)

1. **제어판 v3**를 켭니다. 버전, 로봇 구성, 카메라, 평가 탭의 AI 추론 통신이 "정상"인지 확인합니다.
2. **현장 패널** ① Host
   - 규약 **v2 · Fleet**, 브로커 **관제 PC**, 실행부 **실물 제어판**을 고릅니다.
   - **Host 시작**을 누릅니다.
   - 위쪽 칩에서 Host·제어판·AI 추론·카메라가 정상인지 봅니다.
3. **현장 패널** ② 안전: 로봇 옆에 사람이 있음을 체크하고 **armed 켜기**를 누릅니다.
4. 관제(Fleet)가 명령을 보냅니다.
   - **관제 명령 현황** 표에서 받은 시각, 단계별 시간, 결과·사유를 봅니다.
   - 사유 코드를 누르면 뜻과 할 일이 나옵니다.
   - 관제 PC 브로커에서는 현장 패널이 명령을 보내지 않고 보기만 합니다.
5. **HOLD**가 걸리면 로봇 옆에서 팔·상자·Pinky를 확인하고, ④ HOLD의 세 항목을 체크한 뒤 **HOLD 해제 (현장)**를 누릅니다.
   - 팔이 토크를 켠 채 제자리에 멈춰 있으면, 먼저 제어판의 **[홈 위치로 이동]**을 누릅니다.
6. 끝나면 **armed 끄기**를 누릅니다. 진행 중인 명령이 없을 때 **Host 정지**를 누릅니다.

### 3-5. 관제 없이 시험 (이 PC 브로커)

- 이 PC에 MQTT 브로커가 `127.0.0.1:1883`으로 떠 있어야 합니다.
- 브로커 **이 PC**를 고르고, 실행부는 **모의**(팔을 움직이지 않고 6초 뒤 완료)나 **실물 제어판**을 고릅니다.
- **Host 시작**을 누른 뒤 ③ 미션 명령에서 칸·상자를 골라 실행합니다. 가상 관제가 Fleet처럼 명령, 작업 유지 확인, ACK를 보내고 Pinky 정차를 흉내 냅니다.
- 실물 제어판 실행부는 실제 팔이 움직입니다. 로봇 옆에서만 씁니다.

## 4. 문제가 생기면

| 증상 | 확인 |
| --- | --- |
| 현장 패널이 안 열림 | `omx-host.json`이 있는지, `repo` 경로에서 `uv sync --locked`를 했는지. 터미널에서 `./run.sh`를 실행해 오류 확인 |
| armed가 안 켜짐 | 현장 패널: Host를 먼저 시작. 제어판: 로봇 연결·카메라 확인 상태 |
| 명령이 `executor_version_mismatch`로 거절 | 경로표의 `model_version`·`calibration_version`과 관제 설정이 다름 |
| 평가가 카메라 오류(err 7)로 끝남 | 제어판 로봇 구성에서 카메라 화면 확인, USB 재연결 뒤 저장 |
| AI 추론 "끊김" | GPU 노트북 켜짐·같은 네트워크·제어판 평가 탭 AI 추론 통신 진단 |
| 제어판이 갑자기 꺼짐 | 제어판 창을 닫으면 스스로 꺼짐(작업이 없을 때). 아이콘으로 다시 켬 |

사유 코드의 뜻은 현장 패널에서 코드를 누르면 나옵니다. 제어판 오류 코드(err_code_N)는 제어판 도움말 "오류 코드"에 있습니다.
