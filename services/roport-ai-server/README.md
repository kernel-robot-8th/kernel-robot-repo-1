# roport-ai-server

RO:PORT · GATE CREW — 모델 로딩·추론 API·모델 버전 관리

## 기술 스택

Python 3.12 / FastAPI / Uvicorn / uv / pytest / Ruff. 모델 런타임은 PyTorch 기준이며 모델·CUDA·드라이버 조합 확정 후 추가합니다.

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

```bash
python3 -m pip install uv==0.13.5
uv sync --locked
uv run ruff check .
uv run pytest
uv build
```

API 실행: `uv run uvicorn app:app --app-dir src --host 127.0.0.1 --port 8000`

`/healthz`는 프로세스 상태만 확인합니다. 로봇·DB·모델 준비 상태를 뜻하지 않습니다.

## OMX 추론 UDP 서버 (시험 단계, FFT1-154)

현장 Host PC의 OMX 제어판이 최신 관측(관절 값 + 카메라 JPEG)을 보내면 동작 값 묶음을 돌려줍니다. 로봇 구동·안전 정지는 Host PC 제어판이 맡고, 이 서버는 장비를 제어하지 않습니다.

- 규약 `RPI1`(`src/omx_udp/wire.py`): 메시지를 1200바이트 이하 datagram으로 나누고 전체 크기·CRC32·HMAC-SHA256을 싣습니다. 모든 조각이 모이고 두 검사를 통과한 메시지만 사용하며, 같은 세션의 더 새 요청이 오면 덜 모인 이전 요청은 버립니다. 서버는 밀린 요청 중 세션별 최신 요청만 계산합니다. 다시 요청할 때는 항상 새 `request_id`를 씁니다.
- 종류: `PING→PONG`, `LOAD→LOADED`, `INFER→ACTIONS`, 실패는 `ERROR {code, message}`(`MODEL_NOT_ALLOWED`, `BAD_REQUEST`, `INFER_FAILED`).
- `LOADED` 응답에는 모델의 설정 파일 텍스트 `files`(`config.json`, `train_config.json`)와 모델 시작 자세 `home`이 들어갑니다. 학습 데이터가 없어 `home`을 만들 수 없으면 `home: null`과 `home_error`를 보냅니다. `LOAD`의 `overrides`에 `{"_info_only": true}`를 넣으면 모델을 GPU에 올리지 않고 `files`·`home`만 돌려줍니다(`info_only: true`). Host 제어판은 평가를 준비할 때 이 요청으로 설정과 시작 자세를 받으므로 GPU PC에 SSH로 접속하지 않습니다.
- `INFER`는 `schema_version`(`roport-omx-infer-v1`)·`inference_id`·`observation_seq`(=`request_id`)·`model`·`obs`가 없으면 `BAD_REQUEST`입니다. 응답은 `inference_id·observation_seq·task_id·command_id·execution_id`를 되돌려주고 `model_version`과 `valid_until`(요청의 `deadline`)을 붙입니다. 규약은 팀 AI UDP 설계안(Confluence MFS 10125435)의 원칙을 따르지만 팀 합의 전 초안입니다.
- 보안: tailscale 없이 같은 현장망에서 쓰므로 공유 키(HMAC)와 허용 Host IP로만 받습니다. `0.0.0.0` 바인드는 거부합니다. 키 파일은 소유자만 읽을 수 있어야 하며(`chmod 600`) 저장소에 올리지 않습니다.
- 실행: 운영 중에는 GPU PC의 OMX 제어판(3.13.0 이상)이 서버를 켜고 지킵니다. 설정(`bind`·`port`·`peers`)은 GPU PC의 `~/.config/omx/v3/udp_server.json`에 기억되고, 제어판이 다시 켜지면 서버도 바로 다시 켭니다. 제어판 없이 시험할 때만 직접 실행합니다(GPU PC, 이 디렉토리): `PYTHONPATH=src <제어판 LeRobot 환경의 python> -m omx_udp.server --bind <GPU PC IP> --peer <Host PC IP> --key-file <키 파일> --panel-tools <omx_panel/omx_tools 경로>`. 서버 코드는 표준 라이브러리만 쓰고, 모델 실행은 제어판의 `omx_infer.py`(LeRobot·PyTorch)와 시작 자세 계산 `omx_home.py`를 그대로 씁니다.
- 키 만들기: `python3 -c "import secrets; print(secrets.token_hex(32))" > <키 파일> && chmod 600 <키 파일>`
- 검사: `uv run pytest`는 loopback으로 규약·서버만 확인합니다. GPU 추론(`panel_policy.py`), 실제 망 손실·지연, Host 제어판 연동, 실물 팔은 미검증입니다.
