# OMX 제어판 연동 기준 (v1 Host)

`task/omx-cooperative`의 Host 프로그램이 함께 동작하는 OMX 제어판의 버전과 사용하는 API를 기록합니다. 제어판 코드는 별도 저장소에서 개발하고, 이 문서로 "어느 제어판 버전과 맞물려 검증했는지"를 남깁니다. 통합 점검과 v2 전환 때 기준으로 씁니다.

## 1. 구성

| 이름 | 무엇 | 위치 |
| --- | --- | --- |
| Host PC | 현장 인도장 PC(OMX1·2, 카메라 USB 연결) | - |
| 제어판(Control Panel) | 화면·armed 스위치, 로봇 루프(USB 시리얼), 카메라, AI 추론 UDP 요청 | 별도 저장소 [`tjkoobro-netizen/omx-control-panel`](https://github.com/tjkoobro-netizen/omx-control-panel) |
| Host 프로그램 | 관제 MQTT 창구. 미션 접수·판정·보고, HOLD 관리 | 이 저장소 `services/roport-omx` (`scripts/cooperative_host.py`) |

```
관제 서버 ─MQTT─▶ Host 프로그램 ─HTTP 127.0.0.1:8865─▶ 제어판 ─USB 시리얼─▶ OMX1·2
                    │  (같은 Host PC)                      └─UDP RPI1─▶ AI 추론 서버(GPU)
                    └─ 로컬 파일(HOLD 해제) ◀──────────── 제어판 [HOLD 해제] 버튼
```

- 제어판은 armed를 켤 때 `~/.config/roport/omx-host.json`의 `repo`에서 Host 프로그램을 자동으로 띄웁니다. 현장 실행 폴더는 개발 폴더와 분리한 전용 worktree입니다.
- Host 프로그램은 제어판 API만 호출합니다. 로봇·카메라·AI 추론 서버에 직접 접근하지 않습니다.

## 2. 검증 기준 버전

| 날짜 | 제어판 | Host 프로그램 | 결과 |
| --- | --- | --- | --- |
| 2026-11-17 | v3.16.0 (`02a6246`) | `task/omx-cooperative` + #99 #100 #107 #103 (현장 `field/omx-cooperative`) | 모의 시험·제어판 샌드박스 통과. 실물 팔·카메라 미검증 |

Host가 요구하는 제어판 최소 버전: **3.14.0**(입고만), **3.15.0**(경로표에 OUTBOUND가 있을 때). 낮으면 `executor_error: PANEL_VERSION`.

## 3. Host가 쓰는 제어판 기능

| 기능 | 제어판 | Host 사용처 | 생긴 버전 |
| --- | --- | --- | --- |
| armed 스위치, `source: "mqtt"` 평가는 armed일 때만 | `GET /api/status` → `armed` | 준비 조회, `NOT_ARMED` 거절 | 3.8.0 |
| 상태·결과 | `GET /api/status` → `version`, `job{running, kind, info{repo_id, eval_repo_id, phase}}`, `last_result` | 진행·완료 판정 | 3.8.0 |
| 학습 모델 목록 | `GET /api/train/runs` → `runs[].model_path` | `MODEL_NOT_READY` 거절 | 3.8.0 |
| armed 때 Host 자동 시작 | `~/.config/roport/omx-host.json` | 현장 실행 | 3.11.0 |
| 모델 홈 기준 팔 실측 | `GET /api/arms?model=` → `arms{at_home, gripper, joints, side}` | 준비 조회, 완료·실패 전 최종 확인 | 3.11.0 |
| 복귀하면 시도 종료 | `POST /api/eval/start` `end_on_return: true` → `last_result.last_episode.returned` | 정책 재시도 방지 | 3.12.0 |
| AI 추론 UDP 상태 | `GET /api/status` → `gpu_link.infer{transport, udp{ok, stage, error, at}}` | `AI_LINK_*` 거절(udp·ok true·20초 이내만 준비) | 3.13.0 |
| 평가 뒤 선반 정렬 | `POST /api/eval/start` `shelf_align: true` → `last_result{stage, final, align{ran, ok, slot, on_shelf, stopped_by, released, rest}}` | 선반 적재 판정 | 3.14.0 |
| 홈 복귀 | `POST /api/home/return` `{"source": "mqtt"}` → `job.kind "home"` | `MISSION_FAILED` 뒤 양팔 시작 자세로 | 3.14.0 (armed 조건 3.15.0) |
| 평가 뒤 Pinky 상차 확인 | `POST /api/eval/start` `pinky_check: true`, `shelf_align: false` → `last_result.pinky{ran, on_pinky, on_shelf, pinky, shelf, skipped}` | 하차 판정 | 3.15.0 |
| HOLD 해제 버튼 | journal 옆 `<stem>.host-state.json` / `.hold-request.json` / `.hold-result.json` | Host가 조건 확인 후 해제 | 3.16.0 |

평가 시작 요청 본문(Host가 보냄): `source`, `model_path`, `task`, `episodes: 1`, `episode_s`, `reset_s`, `target_marker`, `end_on_return`, (`shelf_align` | `pinky_check` + `shelf_align: false`), 경로표의 `output_filter`·`filter_gripper`·`eval_options`.

## 4. 현장 설정 파일 (Host PC)

| 파일 | 내용 |
| --- | --- |
| `~/.config/roport/omx-host.json` | Host 실행 폴더(`repo`), `link_config`, `broker_env`, `routes`, `box_marker`, `journal`, `client_id` |
| `~/.config/roport/shelf-01-routes.json` | `(operation, slot_id)`별 모델·작업 문장·시간·필터. 예시 `config/cooperative_routes.example.json` |
| `~/roport-baseline/run/` | journal(HOLD 기록), 관제 연결 설정, 브로커 접속 정보(비밀값, 커밋 금지) |

## 5. 제어판 변경 예정(Host와 함께 바뀌는 것)

| 할 일 | 이유 |
| --- | --- |
| 정렬·상차 확인 칸을 명령에서 받기 | 지금은 설정 2B 고정이라 다른 칸은 `PANEL_ALIGN_WRONG_SLOT` |
| v2 관제 규약 대응(Host 자동 시작 대상, 명령별 시작·조회·취소 API, 모터 정지 관측) | 관제 fleet이 v2 명령을 보냄 |
| 관제 원격 HOLD 해제 표시 | 관제 해제 규약이 정해지면 |

제어판을 바꾸면 이 문서의 2·3절과, 그 기능을 쓰는 Host PR 설명의 "필요한 제어판 버전"을 함께 갱신합니다.
