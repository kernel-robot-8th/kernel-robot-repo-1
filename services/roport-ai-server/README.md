# roport-ai-server

RO:PORT · GATE CREW — 모델 로딩·추론 API·모델 버전 관리

## 기술 스택

Python 3.12 / FastAPI / Uvicorn / uv / pytest / Ruff. 모델 런타임은 PyTorch 기준이며 모델·CUDA·드라이버 조합 확정 후 추가합니다.

## 현재 범위

FastAPI 시작 구조와 분담 제어용 정책 후보 라우터를 제공합니다. 학습된 모델의 로딩·영상 전처리·GPU 실행은 아직 연결하지 않았습니다.
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

## 분담 정책 후보 · source/mock

`dedicated_policy.PolicyRouter(policies, metadata)`에 작업·단계별 provider와 모델/feature/보정 메타데이터를 등록합니다. provider는 `reset()`과 `infer(observation)`을 구현하고, 관절 5개+그리퍼 1개의 위치 후보 묶음을 반환합니다. 이는 LeRobot ACT 연결점이며 등록만으로 학습된 ACT가 준비되지는 않습니다. `/readyz`는 계속 미준비를 반환합니다.

- wire profile: `omx.dedicated/1`, ISO 8601 offset 시각, 최대 60,000-byte strict JSON. 중복 키·비유한 값·잘못된 feature 순서·이전 실행/단계·만료된 관측은 거절합니다.
- `handle(request, now=UTC_epoch_seconds)`와 `handle_datagram(bytes, now=...)`는 직렬로 호출합니다. 단계 revision이 바뀌면 reset을 다시 확인하며 미등록 모델·provider 오류는 `inference.error`입니다.
- `dedicated_udp.serve_once(socket, router, allowed_peer=..., clock=...)`는 호출자가 준비한 UDP 소켓에서 지정 peer의 요청 1개를 처리합니다. import·HTTP 서버 시작만으로 소켓을 열지 않습니다. `allowed_peer`는 IPv4 `(host, port)` 2-tuple 전용입니다(IPv6 미지원). 다른 peer의 패킷은 응답 없이 버리고 경고 로그를 남긴 뒤 `"unexpected_policy_peer"`를 반환하며, 처리한 경우 `None`을 반환하므로 호출자는 예외 없이 루프를 계속합니다. peer 제한은 암호학적 인증이 아니며 현장 네트워크 적용 전 별도 검증이 필요합니다.
- 동기 provider를 강제 중단하지 않습니다. 반환 후 deadline이 지난 후보는 폐기하고 Host도 자체 timeout으로 보류해야 합니다. 모터 구동·완료·Pinky 출발은 AI의 권한이 아닙니다.
