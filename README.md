# roport-control-server

RO:PORT · GATE CREW — 작업 배정·상태 관리·ROS 2 Bridge

## 기술 스택

Python 3.12 / FastAPI / Uvicorn / uv / pytest / Ruff. 작업 상태 저장은 PostgreSQL 기준이며 DB 연결·마이그레이션은 후속 구현입니다. ROS 2 Jazzy Bridge는 API와 별도 프로세스로 실행합니다.

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

API 실행: `uv run uvicorn roport_control_server.app:app --host 127.0.0.1 --port 8000`

`/healthz`는 프로세스 상태만 확인합니다. 로봇·DB·모델 준비 상태를 뜻하지 않습니다.
