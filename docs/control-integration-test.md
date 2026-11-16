# 관제 통합 시험 기준

관련 작업: [FFT1-164](https://fp-team1.atlassian.net/browse/FFT1-164),
[FFT1-180](https://fp-team1.atlassian.net/browse/FFT1-180).

## 브랜치 운용

- 기준 브랜치: `task/control-integration-test` (최신 main에서 생성).
- 초기 통합 PR의 소스: `feature/inyup-baek/control-integration-test`.
- 초기 PR 병합 전에는 feature를 clone한다. task에 아직 구현이 들어갔다고 가정하지 않는다.
- 후속 서비스 연결은 task 병합 후 task에서 새 feature를 만들어 PR로 반영한다.
  보호된 task/main에 직접 push하거나 공유 이력을 재작성하지 않는다.
- 기존 main용 OMX PR #79와 다른 열린 PR은 유지한다. 이 브랜치가 그 PR을 자동 병합/종결하지 않는다.

현재 구현을 함께 시험하기 위한 큰 통합 단위다. main으로 올릴 때는 실제 main/열린 PR 상태를
다시 대조하고 기능별 리뷰 범위와 중복을 정리한다. 전체 차이를 새 구현으로 해석하지 않는다.

## 포함 범위

| 영역 | 현재 상태 |
| --- | --- |
| 관제 웹·실행부 | FastAPI → Unix socket → 단일 Fleet owner. UI 직접 입출고 요청·역할 배정·중단 복구 |
| 주행·자원 | 노드/간선 점유·구간 해제·동시 주행·복귀 중 재배정·N12/Nav2 왕복 |
| 로봇 연결 | 최소 MQTT v2, Pinky 드라이버 재사용, OMX3·단독 팔·공동 팔 어댑터 |
| 서비스·DB | WSS inbox/outbox, 관제 운영 저장소, 예약 출발 정책·명세 갱신·포장품별 수령 |
| 현장 준비 | 현재 소스 배포 폴더·체크섬, 수신 전용 MQTT 점검, Pinky/OMX 설치 매뉴얼 |

고객 웹/카트 패드의 전체 실제 API 연결, 포장 완료 입력→입고 연계와 사용자 진행 조회는 후속이다.
예약 300초 선행 출발은 임시 정책이다. QR·카트 인증은 개발용이며 외부 공개 운영용이 아니다.
OMX 패널 수신 모듈은 외부 패널의 실제 실행 경계에 설치해야 한다. 실제 센서·모델·장비 보정은 미포함이다.

## 빠른 가상 시험

루트 README의 clone 후 `services/roport-control-server`에서 실행한다.

```bash
npm ci --ignore-scripts
npm run prepare:assets
bash tools/start_virtual_ui.sh
```

마지막 URL·키로 접속한다. UI 역할 확인 → 입고 요청 → 요청 처리 시작 → 선반 안착 후 같은
물품 출고 요청 → 고객 도착 후 가상 수령 확인 순서다. `package-small-1`, `package-large-1`로
크기를 구분한다. 시작 자체로 로봇이 출발하지 않는다. Ctrl+C로 생성 프로세스를 종료한다.
각 실행의 로그/config/journal은 `~/roport-trial/fleet/실행시각/`에 보존된다.

자동 검증은 같은 디렉터리에서 다음과 같이 실행한다(Mosquitto 필요).

```bash
uv run python tools/run_virtual_fleet.py --verify --timeout 1200
```

가상 기본 속도 0.08m/s, 실물 생성 설정 0.06m/s다. `field_verified` 자동 승인은 하지 않는다.
실물은 [공통 현장 준비](field/README.md), [Pinky](field/pinky.md), [OMX](field/omx.md)를 따른다.

## 확인한 검증과 한계

2026-11-16 로컬 기준: 관제 전체 892건(skip 0), OMX 155건, 패키지 빌드·Ruff·자산 검사 통과.
마지막 준비 도구 12건과 UI 실행/준비 시험 16건, 패널 수신 12건은 전체와 중복되므로 합산하지 않는다.
서비스 Java는 앞선 구현 검증 37건 및 HTTP/WSS/DB/MQTT 통합 시험 기록이 있다.
센서·모터·Nav2 I/O는 모사했다. 가상 검증을 실물 상하차·공간 충돌/제동거리 검증으로 간주하지 않는다.

시험 결과에는 checkout commit과 변경 여부, 시험 종류(단위/가상/실물), 실행 config와 실패 명령 ID를
기록한다. 조작 키·자격증명·실제 영상은 공개하지 않는다. 실행 중 DB/journal을 지워 초기화하지 않는다.
