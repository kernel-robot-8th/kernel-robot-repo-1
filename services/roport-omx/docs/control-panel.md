# OMX 제어판 연동 기준 (v2 Host, 제어판 3.28.0)

OMX Host 프로그램(v2 기준, `main`)이 함께 동작하는 OMX 제어판의 버전과 사용하는 API를 기록합니다. 제어판 코드는 별도 저장소에서 개발하고, 이 문서로 "어느 제어판 버전과 맞물려 검증했는지"를 남깁니다. 통합 점검 때 기준으로 씁니다.

현장 기준(2026-11-25): 관제 규약은 **v2**(`roport-control-v2`, Fleet), 제어판은 **3.28.0**입니다. v1(`roport-omx-task-v1`, OmxTrial)은 이전 시험 연동으로만 남아 있고 `--protocol v1`(제어판 설정 `"protocol": "v1"`)일 때만 씁니다.

## 1. 구성

| 이름 | 무엇 | 위치 |
| --- | --- | --- |
| Host PC | 현장 인도장 PC(OMX1·2, 카메라 USB 연결) | - |
| 제어판(Control Panel) | 화면·armed 스위치, 로봇 루프(USB 시리얼), 카메라, AI 추론 UDP 요청 | 별도 저장소 [`tjkoobro-netizen/omx-control-panel`](https://github.com/tjkoobro-netizen/omx-control-panel) |
| Host 프로그램 | 관제 MQTT 창구. 이송 명령 접수·판정·보고, 상태 보고, 정지, HOLD 관리 | 이 저장소 `services/roport-omx` (`scripts/cooperative_host.py`, v2 `cooperative_v2.py`, v2 실행부 `shelf_panel.py`. v1 실행부 `panel_executor.py`는 이전 OmxTrial 시험용) |

```
관제 Fleet ─MQTT v2─▶ Host 프로그램 ─HTTP 127.0.0.1:8865─▶ 제어판 ─USB 시리얼─▶ OMX1·2
  (roport/workcell/shelf-01/…)  │  (같은 Host PC)                    └─UDP RPI1─▶ AI 추론 서버(GPU)
Pinky 상태 ─MQTT──────────────▶│
                                └─ 로컬 파일(HOLD 해제) ◀──────────── 제어판 [HOLD 해제] 버튼
```

- 제어판은 armed를 켤 때 `~/.config/roport/omx-host.json`의 `repo`에서 Host 프로그램을 자동으로 띄웁니다(4절). 현장 실행 폴더는 개발 폴더와 분리한 전용 worktree입니다.
- Host 프로그램은 제어판 API만 호출합니다. 로봇·카메라·AI 추론 서버에 직접 접근하지 않습니다.
- v2 명령 단위(command_id)의 접수·정지·결과는 Host가 관제와 주고받고, 제어판에는 지금처럼 평가 시작·상태·정지 HTTP만 씁니다. 제어판에 명령 단위 API를 따로 두지 않습니다(2026-11-22 TK 결정, RO:PORT #135).

## 2. 검증 기준 버전

| 날짜 | 제어판 | Host 프로그램 | 결과 |
| --- | --- | --- | --- |
| 2026-11-17 | v3.16.0 (`02a6246`) | `task/omx-cooperative` + #99 #100 #107 #103 (현장 `field/omx-cooperative`) | 모의 시험·제어판 샌드박스 통과. 실물 팔·카메라 미검증 |
| 2026-11-17 | v3.16.1 → v3.17.0 (`375840a`) | 위 + #107 목표 원 끄기 | 현장 실물: 입고 1회 COMPLETE(정렬), 하차 0.2 설정 6회 중 정책 성공 5회 |
| 2026-11-18 | v3.17.3 (`bebd54f`, 태그) | 위 + #112 #114 (현장 `field/omx-cooperative`) | 모의 시험·제어판 샌드박스·MuJoCo 재확인 통과. 실물 미검증 |
| 2026-11-18 | v3.18.0 (`e50ff19`, 태그) | 위 (넘겨받기 `--end-on-place`·`--place-shadow`) | 제어판 샌드박스 18/18. 실물 단계 확인 예정 |
| 2026-11-19 | v3.19.0 (`8bc9464`, 태그) | 위 | 카메라 front_3·front_4 추가(Host 변경 없음) |
| 2026-11-19 | v3.20.0 (`89c44a7`, 태그) | 위 + #107 `48aad80` · #112 `a9da33e` (현장 `field/omx-cooperative` `379ce03`) | 두 PC 배포, 현장 정렬기 교체. 모의 시험 통과, 실물 미검증 |
| 2026-11-21 | v3.21.1 (`3edb249`) | 위 + #134 (v1, 넘겨받기 기본 켬) | 11/21 넘겨받기 첫 실물 시험에서 찾은 빈틈 수정(평가 오류 뒤 결과 닫기, 제자리 정지 중 관제 홈 복귀 거절). 11/22 Host PC 로컬 브로커 + 시험용 관제 송신기로 입고 확인 |
| 2026-11-23 | v3.21.2 → v3.22.0 | #135 #154 (v2, 현장 `field/omx-cooperative`) | 제어판이 v2 Host를 기본으로 띄움. 11/23 22:48 현장 첫 v2 입고(2B, 상자 16, pinky2)에서 시작 대기 중 MQTT 반복문 정지로 HOLD → #156. 23:20 왼팔이 거의 움직이지 않은 회차 → 제어판 3.22.0 손목 카메라 좌우 확인 |
| 2026-11-24 | v3.23.0 | 위 + #156 #157 (현장 `field/omx-cooperative` `953b359`) | 두 PC 배포. 3.22.1~3.23.0은 시작 단축·떨어뜨림 기록으로 Host 변경 없음. 3.22.3은 11/24 00:28 관제 입고에서 미리 띄운 평가의 넘겨받기가 일어나지 않은 문제 수정. 실물 팔로 3.23.0 전체 미션 재확인은 아직 |
| 2026-11-25 | **v3.28.0** (제어판 저장소 main `93d0041`) | 위 + #169 #171 (`main`, v2) | 두 PC 배포. 3.24.x 출고 쟁반 놓음 판정(Host `--report-unload-placement`, 관제 지원 전까지 끔), 3.25.0 한팔 미션, 3.26~3.27 떨어뜨림 감지·평가 감시·UDP 손실 감시, 3.28.0 떨어뜨림 감지 멈춤 모드(기본 꺼짐)·AI 서버 카운터 이름(#177). 한팔 실물 미션은 아직 |

v3.21.2~v3.28.0은 2026-11-25 제어판 저장소 main에 합쳤습니다(#1~#3). 버전마다의 내용은 제어판 저장소 `CHANGELOG.md`에 있습니다.

Host가 요구하는 제어판 최소 버전: **3.14.0**(입고만), **3.20.0**(경로표에 OUTBOUND가 있을 때, 작업 직전 쟁반 사진 확인이 필요). 낮으면 `executor_error: PANEL_VERSION`. 그보다 새 기능은 제어판 버전을 보고 있을 때만 씁니다: 칸·`pinky_id` 전달은 3.17.0+, 넘겨받기(`end_on_place`)·그림자 판정(`place_shadow`)은 3.18.0+, 놓였음 즉시 완료(`report_placement`)는 판정 근거가 생긴 3.20.0+. 출고 쟁반 놓음 즉시 완료(`report_unload_placement`)는 3.24.1+, 한팔 이송은 3.25.0+.

현장 권장 버전은 **3.28.0**입니다. 3.22.2는 미리 띄운 평가에서 놓음 판정과 복귀 종료가 꺼지므로(3.22.3에서 고침) 쓰지 않습니다. 손목 카메라 좌우 확인(`PANEL_WRIST_CHECK`)은 3.22.0+에서만 나옵니다.

## 3. Host가 쓰는 제어판 기능

| 기능 | 제어판 | Host 사용처 | 생긴 버전 |
| --- | --- | --- | --- |
| armed 스위치, `source: "mqtt"` 평가는 armed일 때만 | `GET /api/status` → `armed` | 준비 조회, v1 `NOT_ARMED`·v2 `not_armed` 거절 | 3.8.0 |
| 상태·결과 | `GET /api/status` → `version`, `job{running, kind, info{repo_id, eval_repo_id, phase}}`, `last_result` | 진행·완료 판정, v2 `robot_state` | 3.8.0 |
| 학습 모델 목록 | `GET /api/train/runs` → `runs[].model_path` | `MODEL_NOT_READY` 거절. 상태 표본 스레드가 30초마다 뒤에서 갱신 | 3.8.0 |
| armed 때 Host 자동 시작 | `~/.config/roport/omx-host.json` | 현장 실행 | 3.11.0 (v2 기본 3.21.2) |
| 모델 홈 기준 팔 실측 | `GET /api/arms?model=` → `at`, `arms{at_home, gripper, joints, side}` | 준비 조회, 완료·실패 전 최종 확인, v2 정지 확인(정지 뒤 버스 읽기 2회의 `at`·관절 비교) | 3.11.0 |
| 복귀하면 시도 종료 | `POST /api/eval/start` `end_on_return: true` → `last_result.last_episode.returned` | 정책 재시도 방지 | 3.12.0 |
| AI 추론 UDP 상태 | `GET /api/status` → `gpu_link.infer{transport, udp{ok, stage, error, at}}` | `AI_LINK_*` 거절(udp·ok true·20초 이내만 준비) | 3.13.0 |
| 평가 뒤 선반 정렬 | `POST /api/eval/start` `shelf_align: true` → `last_result{stage, final, align{ran, ok, slot, on_shelf, stopped_by, released, rest}}` | 선반 적재 판정 | 3.14.0 |
| 홈 복귀 | `POST /api/home/return` `{"source": "mqtt"}` → `job.kind "home"` | `MISSION_FAILED` 뒤 양팔 시작 자세로 | 3.14.0 (armed 조건 3.15.0) |
| 평가 뒤 Pinky 상차 확인 | `POST /api/eval/start` `pinky_check: true`, `shelf_align: false` → `last_result.pinky{ran, on_pinky, on_shelf, pinky, shelf, skipped}` | 하차 판정 | 3.15.0 |
| 안전 정지 | `POST /api/stop` → armed 꺼짐, 팔 내려놓기 | v2 `robot_stop_command`, watchdog 만료, Pinky 이탈 | 화면 [중지]와 같음 |
| HOLD 해제 버튼 | journal 옆 `<stem>.host-state.json` / `.hold-request.json` / `.hold-result.json` | Host가 조건 확인 후 해제 | 3.16.0 |
| 하차 평가에서 목표 원 끄기 | `POST /api/eval/start` `draw_target: false` → `last_result.target_drawn` | 하차 모델은 목표 원 없이 학습됨 | 3.16.1 |
| 명령 칸·Pinky 전달, 놓였음 먼저 보고 | `POST /api/eval/start` `slot: "2B"`, `pinky_id` → `last_result.placement{on_shelf, slot, box, source, observed_at, reported_at}` | 칸별 정렬·상차 확인, 두 단계 보고 | 3.17.0 |
| 정렬 뒤 모델 홈 복귀 | `last_result.align.rest{check, rest_ref: "home"}` | 정렬 뒤 쉬는 자세 판정 | 3.17.1 |
| 안전 정지 홈 이동: 모든 관절 동시 도착, 먼저 손끝 4 cm 들기 | 평가 종료·중지·`/api/home/return` 공통 | 실패 뒤 홈 복귀 경로(충돌 감소) | 3.17.2 · 3.17.3 |
| 넘겨받기, 그림자 판정 | `POST /api/eval/start` `end_on_place: true` 또는 `place_shadow: true` → `last_result.handover{mode, state, end_reason, judge{box{id, flat}}, fallback, fallback_ok, …}`, `last_episode.end_reason` | 놓은 직후 정렬기가 팔을 이어받음, 실패 시 판정(`PANEL_HANDOVER_*`). 선반 판독 불가 때 대체 완료는 판정기가 직접 읽은 마커(`judge.box.id`, 2B 영역·명령 상자만 찾음)가 명령 상자일 때만 | 3.18.0 |
| 하차 직전 쟁반 확인과 직전 사진 기준 판정 | `POST /api/eval/start`(하차) 안에서 직전 쟁반 확인, 비어 있지 않으면 400 `{"refused": "tray_check", "tray_check": {...}}` → `last_result.pinky{tray_reference: "pre_mission", reference_at, tray_reference_state, pre_mission_match}` | 거절은 `PANEL_TRAY_NOT_EMPTY`·`PANEL_TRAY_CHECK_FAILED`(HOLD 없음), 직전 사진 기준이 아니면 `PANEL_PINKY_NO_REFERENCE`(HOLD) | 3.20.0 |
| 놓였음 판정 근거 | `last_result.placement{tips_clear, tip_y, y_out, gripper, observed_ts, observed_clock, eval_end{all_at_home, arms{gripper}}}` | 즉시 완료의 그리퍼 해제·촬영 3초 이내 판정(`grasp_released: true`). 시각 출처는 아래 참고 | 3.20.0 |
| 넘겨받기 제자리 정지 | 정렬기가 안전한 홈 경로를 못 찾으면 `last_result.stage "hold"`, `handover.fallback "hold"`. 그동안 `/api/home/return`(`source: "mqtt"`)은 400 `{"refused": "hold"}` | `PANEL_HANDOVER_HOLD`, HOLD(팔은 사람이 확인). 평가 오류 뒤에도 결과가 `final true`로 닫혀 `HOST_BUSY`에 묶이지 않음 | 3.21.1 |
| 관제 Host 규약 선택 | `omx-host.json` `"protocol"`(`v2` 기본), `"control"`, `"journal_v2"` | 제어판이 `--protocol v2 --control …`과 v2 journal로 Host를 띄움. `--protocol`을 모르는 이전 Host는 v2로 띄우지 않음 | 3.21.2 |
| 손목 카메라 좌우 확인 | 협동 평가 정책 시작 전 그리퍼를 조금 움직여 확인. 판별 실패 시 정책을 시작하지 않고 `err.code 13`(`err_code_13`), `last_result.wrist_check{ok, swapped, why, change, …}` | `PANEL_WRIST_CHECK`(카메라별 변화량을 `detail`에, 팔은 홈, HOLD 없음). v2 관제에는 `failed` `panel_wrist_check`. 출고에서 쟁반 확인보다 먼저 분류하는 것은 #163 | 3.22.0 |
| 카메라 동시 열기 | 협동 구성 카메라를 팔 연결과 함께 엶. 30초 안에 열리지 않으면 `err_code_7` | Host 변경 없음(카메라 4대 약 13초가 팔 연결 뒤로 가려짐). `err_code_7`은 일반 평가 오류(`PANEL_ERR_CODE_7`, 홈 복귀 뒤 실패) | 3.22.1 |
| 평가 프로그램 미리 띄우기 | `GET /api/status` → `warm{on, ready, pid, age_s, error}`, 평가가 그것을 쓰면 `job.info.warm: true` | Host 변경 없음. 쉬는 동안 `job.running`은 false 그대로 | 3.22.2 (판정 꺼짐 수정 3.22.3) |
| 평가 중 상자 떨어뜨림 감지(기록만) | `last_result.drop_watch{mode "record", confirmed, signals[…]}` | 아직 쓰지 않음(제어판 기록 모드). 끝내기·토크 유지는 현장 기록 뒤 결정 | 3.23.0 |

**놓였음 시각(`observed_ts`)의 출처**: 제어판 3.20.0+의 넘겨받기 판정(`source: "eval_judge"`)은 판정에 쓴 위 카메라 프레임의 촬영 시각을 넣고 `observed_clock: "capture"`로 표시합니다. 촬영 시각을 모르면 제어 주기 시각을 넣고 `observed_clock: "control_step"`으로 표시합니다. 이 값은 촬영 시각이 아니므로 "촬영 3초 이내" 근거가 되지 않습니다. 정렬 실행기가 읽은 놓였음(`source: "align"` 등)은 `observed_clock`을 남기지 않습니다. Host는 `observed_clock: "capture"`일 때만 즉시 완료를 보냅니다.

**입고 두 단계 보고(`--report-placement`)**: v2는 기본 켜짐, v1은 기본 꺼짐입니다(#156, TK 2026-11-24). 켜져 있으면 놓음 판정이 상자가 명령 칸에 놓이고 두 손이 빠진 것을 볼 때 바로 `completed`(`evidence: "shelf_placed"`)를 보내 Pinky가 떠나고, 정렬이 끝나면 같은 명령으로 `completed`(`shelf_align`/`shelf_load_not_aligned`) 또는 `failed`+HOLD를 다시 보냅니다. 관제가 1차 보고를 저장·ACK해야 하므로 관제 Fleet #158 이후(현재 `main`)와 함께 씁니다. 그 전 관제에는 `--no-report-placement`를 씁니다. 제어판 Host 실행기(`omx_host.py`)는 이 옵션을 넘기지 않으므로 Host 기본값을 따릅니다.

평가 시작 요청 본문(Host가 보냄): `source`, `model_path`, `task`, `episodes: 1`, `episode_s`, `reset_s`, `target_marker`, `end_on_return`, 입고는 `shelf_align: true`·`fast_release: true`, 하차는 `pinky_check: true`·`shelf_align: false`·`draw_target: false`, 3.17.0+는 `slot`·`pinky_id`, 3.18.0+ 입고는 옵션에 따라 `end_on_place`(기본) 또는 `place_shadow`, 경로표의 `output_filter`·`filter_gripper`·`eval_options`. 한 팔 평가(`arm`, 3.21.0)는 보내지 않습니다(양팔 미션만, 한 팔 명령은 Host가 거절).

## 4. 현장 설정 파일 (Host PC)

| 파일 | 내용 |
| --- | --- |
| `~/.config/roport/omx-host.json` | Host 실행 폴더(`repo`), `protocol`(`v2` 기본), `control`(v2 관제 ID, 기본 `control-01`), `broker_env`, `routes`, `box_marker`, `journal`(공통 필수: 제어판 실행기 `omx_host.py`가 항상 요구), `journal_v2`(v2 journal, 없으면 `<journal>-v2.json`), `client_id`. v1일 때만 `link_config`(site·binding). 예시 `config/omx-host.example.json` |
| `~/.config/roport/shelf-01-routes.json` | `(operation, slot_id)`별 모델·작업 문장·시간·필터·받을 상자(`markers`). v2는 파일 최상위(또는 경로별) `model_version`·`calibration_version`이 관제 Fleet 명령 값과 정확히 같을 때만 시작합니다(다르면 `executor_version_mismatch`로 거절). v2 예시 `config/shelf_routes.example.json`(두 버전 값 포함). `config/cooperative_routes.example.json`은 v1(OmxTrial) 예시라 버전 값이 없어 v2에 그대로 쓰면 안 됩니다 |
| broker 접속 파일(`broker_env`) | `ROPORT_MQTT_*` 접속 정보(비밀값, 커밋 금지) |
| journal | 명령 기록·HOLD·구간 시간(`timing`). 프로토콜마다 따로 쓰며 Host는 다른 프로토콜의 journal을 거절합니다. `/tmp`는 쓰지 않습니다 |

## 5. 제어판 변경 예정(Host와 함께 바뀌는 것)

아래는 계획이며 아직 구현되지 않았습니다.

| 할 일 | 이유 | Host 쪽 |
| --- | --- | --- |
| 출고 쟁반 놓음 판정(3.24.0 예정, 설계만) | 하차를 상차 확인 전에 끝내 시간 단축 | `last_result.placement`(`operation: "OUTBOUND"`) 처리, 판정으로 끝난 출고의 상차 확인 생략 |
| 떨어뜨림 감지 확정 시 종료·토크 유지 | 3.23.0은 기록만. 현장 기록을 본 뒤 TK 결정 | `drop_watch` 확정을 `PANEL_BOX_DROPPED` 계열로 보고 |
| 관제 원격 HOLD 해제 표시 | v2 `hold_release_command` 결과를 제어판 화면에도 | HOLD 해제 결과 파일 공유 |

제어판을 바꾸면 이 문서의 2·3절과, 그 기능을 쓰는 Host PR 설명의 "필요한 제어판 버전"을 함께 갱신합니다.
