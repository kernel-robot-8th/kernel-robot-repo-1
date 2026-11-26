# OMX On-site Control Panel

Host PC(OMX1·2와 카메라가 연결된 현장 PC)에서 관제 통신 기반 미션을 시험·운영하는 웹 화면입니다. 이 PC에서만 열립니다(`http://127.0.0.1:8870`).

설치와 현장 사용 순서는 [OMX 현장 운영 안내](../../docs/omx-operations.md)를 봅니다. 이 문서는 도구 자체의 설명입니다.

## 실행

```bash
bash install_desktop.sh   # 한 번만: 바탕화면·앱 메뉴에 아이콘 생성
./open.sh                 # 아이콘과 같음: 서버가 꺼져 있으면 켜고 브라우저로 연다
./run.sh                  # 서버만 실행(터미널)
```

`run.sh`는 `~/.config/roport/omx-host.json`의 `repo`(Host 코드, `services/roport-omx`)의 uv 환경으로 `server.py`를 실행합니다. Host와 같은 경로표 규칙과 규약 모듈을 씁니다.

## 할 수 있는 것

- **Host 시작·정지**: 규약 v2(기본, `roport-control-v2`, Fleet) 또는 v1(시험 링크). 브로커는 이 PC(`127.0.0.1:1883`) 또는 관제 PC(`broker_env`). 실행부는 실물 제어판 또는 모의(팔·제어판을 쓰지 않고 6초 뒤 완료, 별도 journal)입니다. 패널이 다시 시작돼도 Host는 계속 돕니다.
- **안전**: 제어판 armed 켜기·끄기(로봇 옆 사람 확인 체크), 홈 위치로 이동, 중지. Host 없이 armed를 켜면 제어판이 관제 PC 브로커용 Host를 직접 띄우므로, 이 패널은 Host가 없을 때 armed 켜기를 막습니다.
- **가상 관제(v2, `v2control.py`)**: 미니 Fleet입니다.
  - **이 PC 브로커**: 상태 조회, 입고(store)·하차(retrieve), 정지, HOLD 해제 명령을 보냅니다. 1초마다 작업 유지 확인을 보내고, 받은 결과에 ACK를 보내며, pinky1이 N11에 서 있는 상태를 흉내 냅니다.
  - **관제 PC 브로커**: 보기만 합니다. 실제 Fleet이 명령하고 ACK합니다.
- **관제 명령 현황**: Host journal에서 받은 명령, 단계별 시각, 결과·사유를 실시간으로 보여 줍니다. 사유 코드를 누르면 뜻, HOLD 여부, 할 일이 나옵니다.
- **HOLD**
  - 현장 해제: 제어판 [HOLD 해제]와 같은 파일 채널을 씁니다. Host가 팔·작업 상태를 실측한 뒤 풉니다.
  - 관제 HOLD 해제: v2 `hold_release_command`를 보냅니다(이 PC 브로커).
  - 강제 해제: Host를 `--clear-hold`로 다시 시작합니다.
- **카메라**: 제어판 카메라 감시와 같은 규칙입니다. USB가 바뀌어 저장된 카메라가 없어지면 자동으로 다시 찾습니다. 화면을 확인한 뒤 [확인 후 저장]을 누릅니다.

## 설정과 기록

- 설정은 제어판 Host 실행기와 같은 `~/.config/roport/omx-host.json`을 씁니다. 예시는 [`config/omx-host.example.json`](../../config/omx-host.example.json)입니다.
  - v2 관제 ID: `control`(기본 `control-01`)
  - v2 journal: `journal_v2`
  - 경로표: `routes`. 경로표의 `model_version`·`calibration_version`이 명령에 들어갑니다.
- 브로커 주소·TLS·계정은 `broker_env` 파일(`ROPORT_MQTT_*`)에서 읽습니다.
  - 비밀번호는 화면이나 명령줄에 나오지 않고, Host 프로세스 환경변수로만 넘어갑니다.
  - 이 파일과 CA 인증서는 저장소에 넣지 않습니다.
- 환경변수
  - `TESTPANEL_PORT`(기본 8870)
  - `TESTPANEL_PANEL_URL`(제어판, 기본 `http://127.0.0.1:8865`)
  - `ONSITE_ALIGN_RUNS`(선반 정렬기가 남긴 front_2 사진 폴더. 없으면 선반 그림 없이 칸 버튼만 보임)
- 기록은 journal 폴더(예: `~/roport-baseline/run/`)에 남습니다.
  - Host 로그: `testpanel-host-*.log`
  - v2 이벤트: `testpanel-v2-*.jsonl`
  - v1: `testpanel-mission-*.jsonl`
