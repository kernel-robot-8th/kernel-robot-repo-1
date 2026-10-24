# roport-ai-server

RO:PORT · GATE CREW — 모델 로딩·추론 API·모델 버전 관리

## 기술 스택

Python 3.12 / FastAPI / Uvicorn / uv / pytest / Ruff. 모델 런타임은 PyTorch 기준이며 모델·CUDA·드라이버 조합 확정 후 추가합니다.

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

```bash
python3 -m pip install uv==0.13.5
uv sync --locked
uv run ruff check .
uv run pytest
uv build
```

API 실행: `uv run uvicorn roport_ai_server.app:app --host 127.0.0.1 --port 8000`

`/healthz`는 프로세스 상태만 확인합니다. 로봇·DB·모델 준비 상태를 뜻하지 않습니다.

## 상세 설계 기준

- [레포별 기술 스택·통신·CI 계획](https://github.com/kernel-robot-8th/kernel-robot-repo-1/blob/main/docs/technology-stack.md)
- [LeRobot·OMX 버전 및 데이터 호환성](https://github.com/kernel-robot-8th/kernel-robot-repo-1/blob/main/docs/omx-compatibility.md)

문서의 현재 구현과 후속 개발 기준을 구분합니다. 실장비·GPU 확인 전 의존성 업그레이드를 확정하지 않습니다.
