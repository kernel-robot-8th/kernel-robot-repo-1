# roport_line_nav

IR 라인 노드 경로 주행(Jira FFT1-117). 물류 Pinky와 스마트카트가 같이 쓴다.
노드 그래프는 [roport_gz_world/layout/roport_airport.yaml](../roport_gz_world/layout/roport_airport.yaml)을 읽고,
IR 판정·기체별 임계값은 [roport_pinky_line](../roport_pinky_line/)(라인 추종 v0)을 쓴다.

실로봇에서는 아직 확인하지 않았다. 단위 테스트는 2D 시뮬레이터(`test/sim.py`, 실측 테이프 배치 + IR 모델)로 한다.
Gazebo에는 IR 센서가 없다. 시뮬레이터 결과는 실로봇 주행 검증을 대신하지 않는다.

## 동작

| 단계 | 내용 |
| --- | --- |
| 경로 | 노드 id 목록(예: `N16 N14 N02 N10 N18 N01`). 이웃 노드끼리만, 중간 U턴 없음, 마지막은 가로선이 있는 정차 노드 |
| 경유 정차 | 중간에 가로선 노드가 있으면 그 노드의 도착 동작(상차 대기·후진 정차·handoff)을 한 뒤 이어서 출발. 예: `N16 N14 N02 N10 N18 N01 N18 N07 N23 N19 N11` = P5 → P4 상차 → P7 상차(이벤트 `via`) |
| 속도 | `{"cmd":"speed","base_speed":0.05,"rotate_speed":0.5}`로 주행 중에도 변경. 범위 = `min_speed`~`max_linear`(기본 0.015~0.06 m/s), `rotate_min_speed`~`max_angular`(0.15~0.6 rad/s). 콘솔을 다시 켜면 launch 값으로 돌아감 |
| 직선 | 라인 추종 v0와 같은 PD 조향, 기본 0.03 m/s |
| 노드 앞 감속 | 이동 거리가 **예상 노드 위치 − `approach_lead_cm`(기본 5 cm)**를 넘으면 `approach_speed`(기본 0.03 m/s)로 줄인다. 회전 전 전진도 이 속도, 후진은 이 속도 이하. **직진 통과 노드(N15·N23 등)는 다가갈 때·지날 때 감속하지 않는다**(`pass_full_speed`, 기본 켬). 출발 위치를 몰라 예상 위치가 없는 구간은 처음부터 이 속도. `base_speed`가 더 느리면 `base_speed`. 속도 명령으로 `base_speed`를 바꿔도 그대로(launch 파라미터) |
| T자 양쪽 확인 | 줄기 쪽에서 T자(왼쪽·오른쪽 갈래 둘 다, 예: P5→N14·P1→N09·P7→N19)로 들어가면 왼쪽·오른쪽 센서가 둘 다 `tee_window_cm`(1.5 cm) 안에서 선을 봐야 노드로 본다(`tee_both`, 기본 켬). 2026-11-07 d660: 왼쪽 센서만 0.3 cm 테이프 가장자리를 밟아 N14를 6 cm 앞에서 오감지 → 회전 실패. 실로그 16건의 진짜 T자 감지는 모두 양쪽이 켜짐(각 1.0~2.8 cm). 한쪽 갈래(모서리 N02·직진 분기)는 그대로 한쪽만 보면 감지 |
| 속도 맞춤 kp(시험판) | 직선 추종 속도가 `gain_ref_speed`(기본 0.06 m/s)를 넘으면 `kp × 속도/gain_ref_speed`(0.07 → 1.17, 0.08 → 1.33). `kp`는 rad/s라 빠를수록 1 cm당 방향 수정이 줄어든다. 2026-11-07 d660 0.08 m/s: 회전 뒤 약 10° 비스듬히 30 cm 끌려가다 반대쪽으로 넘어가 N15를 6 cm 앞에서 옆선 오감지. 0.06 이하와 노드 앞 감속 구간은 `kp` 그대로(`kp_speed_scale`, 기본 켬). 2D 시뮬은 실로봇보다 훨씬 빨리 선에 맞춰져서(3~5 cm) 값은 실로봇 시험으로 정한다. 실로봇 미검증 |
| 노드 감지 | 정차 노드 = IR 3개 모두 테이프. 그 밖의 노드 = 가운데 + 옆 선 쪽 채널이 테이프. **odom 이동 거리가 예상 거리 ±30%(+3 cm) 안일 때만** 노드로 인정(2026-11-03 결정, 테이프 추가 없음). 경로 안에서 이어지는 구간(회전·통과·상차·주차 뒤)은 창 시작을 **예상 위치 − `gate_early_cm`(5 cm)**보다 앞당기지 않는다(2026-11-07 결정). 실로그 61건의 진짜 감지는 예상 −3.4~+2.9 cm이고, N14→N02 모서리는 진짜 18.3~18.5 cm(0.03, 5회)인데 0.06·0.08에서 왼쪽 센서가 본선 가장자리를 밟아 10.9~11.2 cm에서 오감지(3회, 회전 실패)했다. 사람이 놓은 첫 구간은 출발 위치가 가정값이라 비율 창만 쓴다(예외: `start_cross`는 IR 3개를 교차선에 맞춰 놓아 위치를 아는 구간으로 보고 `gate_early_cm`도 적용) |
| 직진 노드 | 방향을 유지하며 5 cm 통과(옆 선 쪽으로 끌려가지 않게) |
| 90° 회전 | 옆 선 감지 → 4.2 cm(IR ↔ 바퀴 축 3 + 테이프 반폭 1.2) 더 직진 → 제자리 회전. 60° 이상 돌고 가운데 채널이 선 위·좌우 오차 0.25 이하이면 다음 구간. 120°까지 못 찾으면 정지 |
| N01(P4)·N11(P7)·N21·N22(02 인도장, layout `arrival: load`) 도착 | 가로선에서 정지 → 반시계 90° → **상차 완료(`load_done`)까지 대기** → 반시계 회전해 진입선을 찾음(합계 약 180°). 정밀 도킹 값이 있으면 아래 「정밀 도킹」 순서 |
| P1·P2·P5·P6 도착 | 가로선에서 정지 → 반시계 회전으로 진입선 확인(172° 이상, 210°까지) → 0.02 m/s 후진 → IR 3개가 가로선을 보면 정지. 12 cm 넘게 후진하면 정지 |
| N12 도착 | 정지하고 `handoff`(Nav2 전환 가능) 보고. Nav2 주행은 범위 밖 |

노드 앞 감속 근거(실로봇 d660, 2026-11-06, P5 → P4): 0.06 m/s에서는 회전 전환 뒤 0.9 cm 더 가서(0.03 m/s는 0.5 cm)
90° 회전이 107~117°에서 끝나고(0.03 m/s는 70~93°) 다음 구간을 약 20° 비스듬히 출발했다. N14→N02(24 cm)에서 왼쪽 센서가
직선 테이프 가장자리를 밟아 10.9 cm(창 10.9~28.7 cm 안)에서 모서리로 잘못 보고, 회전 120° 안에 선이 없어 정지했다.
0.03 m/s는 같은 구간을 18.4 cm에서 맞게 감지하고 완주했다. 그래서 노드 근처만 0.03 m/s로 가고 거리 창(±30%)은 그대로 둔다.
`post_turn_slow_cm` 10 cm(2026-11-10 사용자 결정): 노드에서 회전한 직후 구간은 처음 10 cm를 `approach_speed`(0.03) 이하로 간다. 회전 뒤 선 옆으로 0.5~1 cm 밀린 채 0.06~0.08로 가면 옆 IR이 본선 가장자리를 11.5~15.9 cm까지 밟아 감지 창(노드 5 cm 앞)과 겹쳐 옆선으로 오감지했다(11-07 17:55·11-09 21:54). 0.03 실로그 5회는 가장자리 타기가 있어도 8.5 cm 전에 끝났다. 직진 통과(PASS) 뒤에는 감속하지 않는다. 0이면 끈다.

`approach_lead_cm` 5 cm(2026-11-07 사용자 요청 3~5 cm 중 큰 값): 실로그 61건의 진짜 노드 감지 위치는 예상 위치의 −3.4~+2.9 cm였다. 5 cm면 모두 감속한 뒤 노드를 만나고, 3 cm면 일찍 오는 노드(−3.1·−3.4 cm)를 0.06 m/s로 만난다. 처음 기준(감지 창 시작 − 5 cm)은 긴 구간(N05→N07)에서 노드 약 30 cm 전부터 감속해 바꿨다.
감속 뒤 0.06 m/s 실로봇 완주는 아직 확인하지 않았다.

## 정밀 도킹(FFT1-112, 실로봇 미검증)

상차 노드(N01·N11·N21·N22)마다 세 값을 둔다. 값이 없으면 {0, 90, 0} = 위 표의 동작과 같다.

| 값 | 뜻 | 범위 |
| --- | --- | --- |
| `advance_cm` (a) | ① 가로선에서 멈춘 뒤 접근 방향으로 더 감(음수 = 후진) | −6 ~ 6 |
| `rotate_deg` (r) | ② 반시계 회전 | 60 ~ 120 |
| `move_cm` (b) | ③ 회전 뒤 새 방향으로 직진(음수 = 후진) | −6 ~ 6 |

순서: 가로선 정지 → (a ≠ 0) a 이동 → 반시계 r° → (b ≠ 0) b 이동 → 상차 대기 → `load_done` → (b ≠ 0) −b 이동 → 아래 되돌림 → 끝·경유.
- a = 0: 반시계 회전해 진입선 찾기(창 = (180 − r) ± 30°, r = 90이면 `turn_min_deg`~`turn_max_deg` 그대로).
- a ≠ 0(2026-11-09 사용자 결정): 시계 r°(선 안 찾음, 접근 방향을 다시 봄) → −a 이동(접근선 따라) → 반시계 회전해 진입선 찾기(창 = 90 + `turn_min_deg` ~ 90 + `turn_max_deg`, 기본값 경로의 합계와 같음). 축이 기본 정지 자세로 돌아오므로 다음 구간 센서 위치 = 2·`ir_ahead_cm` + 테이프 반폭(a와 무관). a > +4.5 cm에서 되돌리지 않고 바로 돌아보면 IR이 가로선 위에 남아 진입선을 구분하지 못했다(1차 구현 시뮬 a +6에서 24번 중 22번 정지).
- 이동은 먼저 0을 보내 odom이 멈추길 기다린 뒤(구동 지연으로 남은 움직임을 흘려보냄. odom이 새로 들어온 샘플끼리만 비교해 2번 연속 0.02 cm·0.2° 미만이면 멈춤, 1.5 s 안에 안 멈추면 정지) 그 자세에서 부호 있는 odom 거리로 잰다. 방향 유지(`heading_kp`), 속도 = `approach_speed` 이하, 끝으로 갈수록 `min_speed`까지 감속. 반대로 2 cm 밀림·시간 초과는 오류 정지.
- 관제 시험(`roport_pinky` `trial.launch.py`)도 같은 line_nav라 `~/roport_dock.yaml`에 값이 있으면 관제 시험의 상차 자세(N01·N11·N21·N22)도 바뀐다. 파일이 없으면 기존과 같다. 관제 시험 전에 파일 내용을 확인한다.
- 실로봇 시험 주의: 후진 이동(a < 0, b < 0, 상차 뒤 −b·−a)은 장애물 감지 없이 최대 약 6 cm 움직인다. 상차 노드 둘레에 6~8 cm 여유를 두고 OMX 팔이 대기 자세인지 확인한다. 바퀴를 띄운 상태 → 바닥 저속 순서로 하고 정지는 `stop` 명령·Ctrl+C.
- 저장: 노드 파라미터·launch 인자 `dock_file`(기본 `~/roport_dock.yaml`, 빈 값 = 메모리만). 시작 때 읽고, 없거나 깨지면(형식·범위·상차 노드가 아닌 값이 하나라도 있으면 전체) 경고 후 기본값. `dock` 명령 때 임시 파일 + rename으로 저장한 뒤 적용한다(저장 실패면 적용 안 함).
  ```yaml
  version: 1
  nodes:
    N11: {advance_cm: 1.5, rotate_deg: 96.0, move_cm: 2.0}
  ```
- 명령: `{"cmd":"dock","node":"N11","advance_cm":1.5,"rotate_deg":96,"move_cm":2}`, 지우기 `{"cmd":"dock","node":"N11","reset":true}`. 주행 중(`idle`·`arrived`·`handoff`·`stopped`·`error`가 아닐 때)에는 거부. `line_nav/state`의 `dock`에 기본값이 아닌 노드 값이 나온다.
- 화면 추정(`pose_est`)은 상차 도착 동작 중 가로선 정지 위치(노드 중심에서 접근 반대로 테이프 반폭 + `ir_ahead_cm`) + a·r·b를 보인다. 정지 밀림(구동 지연)은 모른다.
- 화면 추정은 상차 뒤 되돌림(시계 r°·−a)도 따라간다.
- 2D 시뮬(0.03·0.06·0.08 m/s, 구동 지연 0·0.15·0.2 s): 상차 대기 때 바퀴 축이 목표(dock_zone `Core.plan`과 같은 정의, 밀림은 같은 조건 기본값 주행에서 잼) 안. N11 {2, 96, 2}·N01 {−2, 84, −3}은 최대 0.76 cm·1.8°, 실측 요청값 N01 {3.7, 90.4, 2.6}·N11 {−4.6, 88.8, 2.8}은 최대 0.70 cm·1.8°. a −6~+6 × r 60·90·120 × b ±6 × 상차 노드 4개 × 지연 3가지(864회, CI 밖 스크립트로 1회 확인. CI에는 범위 끝 값 96회와 a = +5·+6 48회만)에서 상차 뒤 진입선을 찾아 P5로 복귀.

## 교차선 위 출발(`start_cross`, 실로봇 미검증)

`{"cmd":"start","route":["N07","N18","N01"],"start_cross":true,"start_facing":"S"}` = IR 3개가 출발 노드를 지나는 선(교차선) 위에 있고 로봇은 `start_facing`(E·N·W·S)을 본다(바퀴 축은 노드 중심에서 뒤로 `ir_ahead_cm`).
`ir_ahead_cm` 전진(축을 노드 중심으로) → 첫 구간 방향이 다르면 선 찾기 회전(좌·우 `turn_min_deg`~`turn_max_deg`, 반대 `back_min_deg`~`back_max_deg`) → 첫 구간. 출발 위치를 아는 구간이라 센서 위치 = `ir_ahead_cm`, 감지 창 시작 = 예상 − `gate_early_cm`.
출발 노드가 가로선 노드이거나, 보는 방향을 가로지르는 선이 없거나, `start_offset_cm`과 같이 주면 거부. `start_facing`은 `start_cross`와만 쓴다.

## 옆선 켜짐 확인(FFT1-163 이어서, 실로봇 미검증)

옆선 노드(모서리·분기·직진 통과의 옆선)는 그 옆 IR이 **감지 창 시작(`min_ok`) 뒤에 꺼짐 → 켜짐으로 바뀐 run**일 때만 노드로 본다(`side_rise_only`, 기본 켬). 창 시작 전부터 켜져 있던 run(본선 가장자리 타기)은 무시하고 이벤트 `side_ignored`(run 시작 거리·창 시작)를 남긴다. `side_rise_off_samples`(2) 샘플 연속으로 꺼지면 run이 끝난 것으로 본다(1샘플 꺼짐은 깜빡임으로 보고 같은 run). 가로선(IR 3개)·감지 창·`node_confirm`은 그대로이고 조건을 하나 더한 것이라 판정 강화다.

- 근거(실로봇 d660 0.08 m/s, 2026-11-09 21:54 N02→N14): 회전 직후 오른쪽으로 밀려 왼쪽 IR이 8.0 cm부터 본선 가장자리를 밟음(raw 650~1300, 임계 근처) → 창(14.8 cm)이 열리자 15.1 cm에서 오감지 → 3 cm 이른 회전 → `line_not_found after 120°`. 진짜 옆선은 18.3 cm에서 꺼짐 → 켜짐(raw 1005 → 238). 같은 구간 0.03 m/s는 5/5 정상(18.3~18.5 cm).
- 실로그 개루프 재생(11-04~11-09, 33파일·173구간, 기록된 IR 판정·odom을 그대로 넣고 판정만 비교): 170구간은 예전·새 규칙 결과가 같다(감지 위치는 기록과 최대 0.3 cm 차이). 다른 3구간은 모두 실제 오감지였다.
  - 11-09 21:54 N02→N14: 예전 15.4 cm → 새 규칙 18.5 cm(진짜 옆선).
  - 11-07 17:55 N14→N02, 16:57 N14→N15: 가장자리 run(raw 약 1000~1200)이 꺼지지 않은 채 진짜 옆선(raw 300~500)으로 이어짐 → 새 규칙은 오감지하지 않지만 **진짜 노드도 감지하지 못한다** → 노드를 지나 `node_missed`로 정지한다(오감지 뒤 회전 실패 대신 안전 정지).
  - 진짜 감지 중 새 규칙에서 놓친 것 0건. 진짜 옆선 run은 창 시작보다 0.5 cm 이상 뒤(98구간, 최소 0.5 cm: 11-05 N19→N09).
  - 재생의 한계: 명령이 기록과 달라지면 이후 궤적도 달라지므로 기록된 직진 구간(추종 + 회전 전 전진)까지만 판정을 비교한다. 그 뒤의 결말(위 두 건의 정지)은 추론이다. 11-04~11-05 기록은 당시 그래프(v13), 그 뒤는 v14로 재생했고, 감지 창은 지금 코드(gate_early·tee_both 포함)로 계산했다.
- T자(`tee_both`)에는 기본으로 쓰지 않는다(`side_rise_tee` 꺼짐). T자는 반대쪽 IR이 `tee_window_cm` 안에 함께 켜져야 해서 한쪽 가장자리 타기는 이미 가로선에서 맞게 감지되고, 켜짐 확인을 더하면 그 경우가 놓침이 된다. 실로그의 진짜 T자 37건은 run이 창 시작보다 4.3 cm 이상 뒤라 켜도 결과는 같았다.
- 약점: 노드가 odom으로 예상보다 5 cm 넘게 일찍 오면(엔코더 과소) 진짜 옆선 run이 창 시작 전에 시작해 놓칠 수 있다 → `node_missed` 정지. 2D 시뮬 엔코더 −7%(N02→N10 88.9 cm)에서 일어남. 실로그의 진짜 감지는 예상 −3.4 cm 안이었다.
- 2D 시뮬은 PD가 실로봇보다 빨리 선에 맞춰져 가장자리 타기가 재현되지 않는다. 그래서 실행부가 받는 왼쪽 IR 판정만 8~16 cm 동안 켜짐으로 덮어써(결함 주입) 시험했다. 결과: 0.03·0.06·0.08 m/s × 지연 0·0.15·0.2 s에서 예전 규칙은 9/9 오감지 → `line_not_found`(실로봇과 같음), 새 규칙은 9/9 완주(주입 없을 때와 같은 위치에서 감지).
- 파라미터(노드): `side_rise_only`(true), `side_rise_off_samples`(2), `side_rise_tee`(false).

## 라인 유실

라인 유실 복구(GAP 직진·SEARCH 회전)는 2026-11-15 사용자 결정으로 없앴다. 직선 추종 중 IR 3개가 모두 선을 놓치면 0 명령으로 멈추고, `lost_timeout`(1 s) 안에 다시 못 잡으면 `line_lost on A→B` 오류로 정지한다(main과 같은 동작). 테이프 끊김·벗겨짐은 현장에서 관리한다.

오류(`error`)로 멈추는 경우: 예상 거리를 지나도 노드 없음(`node_missed`), 직선 추종 중 라인 놓침 1초 이상(`line_lost on A→B`), 회전 중 선을 못 찾음,
후진 초과, IR 끊김 0.3초(`no_ir`), odom 끊김 0.5초(`no_odom`). 정지·오류 때 0 속도를 한 번 발행하고 그 뒤로는 발행하지 않는다.

경로가 끝나면 로봇은 다음 출발 방향을 보고 선다. 상차 노드는 진입선 쪽, 대기·충전 칸은 칸 바깥쪽이다.
그래서 다음 경로를 그 노드에서 바로 시작할 수 있다.

이어서 출발: 실행부는 마지막으로 끝난 노드·방향·센서 위치를 기억한다.
- 끝난 노드에서 다시 출발하면 그 센서 위치로 첫 구간 거리 창을 정한다(상차 노드 7.2 cm, 대기 칸 1.2 cm).
- 첫 구간이 로봇이 보는 방향과 반대이면(예: N12 handoff 뒤 `N12 → N08 …`) 먼저 반시계로 반 바퀴(172~210°) 돌아 진입선을 찾는다(이벤트 `pre_rotate`).
- 정지·오류 뒤에는 기억을 지운다. 사람이 로봇을 옮긴 경우 첫 구간 방향으로 세우고 정지한 뒤 시작한다.
- 기억이 없으면 가로선 노드 출발은 센서가 가로선 위(0 cm)에 있다고 보고 거리 창을 쓴다. 그 밖의 노드는 첫 구간 최소 거리 제한이 없다.

## 일시정지·장애물 정지·대기(FFT1-172, 2D 시뮬만 — 실로봇 미검증)

**시연용 소프트웨어 정지, 안전 인증 장치 아님.** Pinky 라이다(RPLIDAR C1)·이 코드는 안전 인증을 받지 않았다. 사람·물체 보호는 현장 감독과 `stop`·Ctrl+C로 한다.

동작은 산업용 라인 AGV 관행을 따른다. 보호 영역에 물체가 들어오면 멈추고, 비면 약 3 s 뒤 다시 간다(Dematic 등). 다가오는 사람·물체를 피해 뒤로 가지 않고 **멈춰 기다린다**. 2026-11-15 사용자 결정으로 다음을 없앴다.
- 수동 후진 `back{to}`(상태의 `back_targets`·`back_why`도 없음)
- 장애물 양보 후진과 그 후진 상태 피드백 제어. 관련 파라미터·상태 `reverse`·이벤트 `yield_*`·속도 키 `reverse_speed`도 없다. `speed`에 `reverse_speed`를 주면 무시하고 결과 문구에 알린다. 그 키만 주면 거절한다. P5·P6 주차 후진(`reverse_speed` 설정, 0.02 m/s)은 그대로다.

| 명령 | 동작 |
| --- | --- |
| `{"cmd":"pause"}` | 0 속도, 상태 `paused`, `pause_reason` `manual`. 지금 단계·구간·이동 거리·감지 창 기억 유지, 시간 제한 없음. odom이 멈추면(\|v\| < 0.003 m/s, \|ω\| < 0.03 rad/s가 0.3 s) `pause.settled` = true·이벤트 `paused_settled`. 장애물 정지 중이면 수동 일시정지를 더해 자동 재개를 막는다 |
| `{"cmd":"resume"}` | 수동 일시정지에서만. 멈춘 단계를 그대로 이어 감(시간 초과 기준은 멈춘 시간만큼 밀림). **장애물 정지 중에는 거절**(`last_command.detail`에 이유) |
| `{"cmd":"stop"}` | 언제나 즉시 0 속도(장애물 정지 중에도) |

장애물 감시 `obstacle_mode`(launch·노드 파라미터, 기본 **off**):
- `off`: 예전과 같음. `scan`을 구독하지 않는다(2D 시뮬·관제 가상 시험 영향 없음).
- `monitor`: 거리만 계산해 `line_nav/state`·route CSV에 남기고 동작은 그대로. 감시 단계에서 멈췄을 상황이면 `obstacle.why` = `monitor: would stop (…)`.
- `stop`: 아래 규칙으로 멈춤.

거리: `scan`(LaserScan, sensor-data QoS)의 유효 점(range_min ≤ r ≤ range_max)을 TF `base_link` ← scan frame으로 옮긴다(라이다가 yaw = π로 붙어 있어 원시 각도를 쓰면 앞뒤가 바뀐다). `front_cm` = |θ| ≤ `sector_deg`(20°) 점의 (x − `front_edge_cm` 4.2) 최솟값, 점이 없으면 null. `rear_cm` = |θ − π| ≤ 20° 점의 (−x − `rear_edge_cm` 7.5) 최솟값은 **감시용으로만** 계산한다(같은 반복 안이라 비용이 거의 없음, 주차 후진·벽 오감지 검토용 CSV). 동작에는 쓰지 않는다. TF를 못 구하면 scan을 버린다(stop 모드는 `scan_stale`로 멈춤). 시각은 받은 때(monotonic)를 쓴다.

| 규칙 | 내용 |
| --- | --- |
| 감시 단계 | `follow`·`pass`(선 따라 앞으로)만. 회전 전 전진·회전·상차 대기·도킹 이동·주차 후진·교차선 출발 이동·수동 `paused` 중에는 정지 판정 안 함(거리는 보고) |
| 정지 | `front_cm` < `stop_cm`(25) → 0 속도, `paused`, `pause_reason` `obstacle`, 이벤트 `obstacle_stop`. 구간·이동 거리·감지 창 그대로. 물체가 더 다가와도 멈춘 채 기다린다 |
| scan 끊김 | scan이 없거나 `scan_max_age_s`(0.5 s)보다 오래됨 → `pause_reason` `scan_stale`(fail-safe). 새 scan이 오면 재개 규칙 |
| 앞 점 없음(R1, 사용자 결정 안전 쪽) | 감시 단계에서 scan은 오는데 앞 부채꼴에 유효 점이 없는 상태가 `scan_max_age_s`(0.5 s) 넘게 이어지면 정지 → `pause_reason` `scan_stale`, `obstacle.why` `front_no_points`. 시연 공간은 사방 벽이 range_max 안이라 점 없음은 근접 가림·렌즈 가림·드라이버 이상일 수 있음. 재개는 scan 끊김과 같고, **유효 front** ≥ 30 cm가 3 s 이어져야 함(점 없음은 빈 것으로 보지 않음) |
| 재개 | `front_cm` ≥ `stop_cm` + `clear_margin_cm`(30) 가 `resume_delay_s`(3 s) 이어지면 멈춘 단계 그대로(이벤트 `obstacle_clear`). 앞 부채꼴에 점이 없으면(range_min 안에 붙은 물체일 수 있음) 비었다고 보지 않는다. 수동 일시정지가 함께 걸려 있으면 장애물 정지만 풀리고 `paused`(`manual`)로 남는다 |
| 사각 의심(계약에 더함) | 장애물 정지 중 마지막 **유효** front가 `blind_near_cm`(15) 미만이었는데 다음 유효 scan이 재개 거리(30 cm) 이상이면(사이에 점 없음 scan이 있어도) 자동 재개하지 않고 수동 재개만(`pause_reason` `manual`, `obstacle.why`에 이유). range_min 안에 들어간 작은 물체는 광선이 무효가 되고 뒤 벽이 보여 앞이 빈 것처럼 나오기 때문(2D 시뮬 8·15 cm/s 물체, 「가까움 → 점 없음 → 멀리」 순서 포함). 앞 끝에서 약 9 cm 사각은 **2D 시뮬 range_min 15 cm로 계산한 값**이고(라이다가 축 뒤 1.7 cm라 앞 끝까지 5.9 cm), 실제 RPLIDAR C1 range_min은 미확인이다. 가까운 물체를 한 번에 들어 올려 치워도 같은 규칙이 걸린다. 조금씩 물러나면(유효 scan마다 15 cm 넘게 뛰지 않음) 자동 재개. `blind_near_cm` 15는 없앤 `yield_cm`(계약 15 cm) 값을 그대로 옮긴 것이다. 0이면 이 규칙을 끈다 |
| 오래 막힘 | 장애물 정지 `warn_after_s`(30 s) → `obstacle.warn` true, 이벤트 `obstacle_warn` 한 번 |

상태 보고(`line_nav/state`, 콘솔·CSV 계약):
```json
{"state": "paused", "pause_reason": "obstacle",
 "pause": {"from": "follow", "settled": true, "manual": false},
 "obstacle": {"mode": "stop", "front_cm": 23.4, "rear_cm": 160.2, "scan_age_s": 0.05,
              "phase": "blocked", "blocked_s": 4.2, "warn": false, "why": null}}
```
- `pause_reason`: null | `manual` | `obstacle` | `scan_stale`(장애물 정지가 걸려 있으면 그 이유, 아니면 수동이면 `manual`). `obstacle.phase`: `clear` | `blocked` | `resume_wait`.
- 이벤트: `obstacle_stop`·`obstacle_clear`·`obstacle_warn`, 수동 `pause`·`resume`·`paused_settled`.
- route CSV 끝 칸: `front_cm`·`rear_cm`·`scan_age_s`·`obstacle_phase`·`pause_reason`.
- LCD: 수동 일시정지·장애물 정지·`scan_stale` 모두 PAUSED(노랑 고정, 추천값). 둘째 줄은 `obstacle 23 cm`·`lidar scan stale`·`lidar no points`.
- 관제 trial_driver(`roport_pinky`, 바꾸지 않음)는 `paused`를 「주행 중」으로 보지 않는다. 그래서 장애물 정지 중 관찰값은 `line_enabled` false·`drive_owner` `none`인데, line_nav는 3 s 뒤 스스로 다시 출발할 수 있다. 구동 중재 lease는 `line`을 유지하므로 cmd_vel 경합은 아니다. trial.launch는 `obstacle_mode`(기본 off)만 넘긴다. **관제 시험(trial)에서는 백인엽 님과 관찰값을 맞추기 전까지 `obstacle_mode:=stop`을 쓰지 않는다.**
- 파라미터(노드·launch): `obstacle_mode`, `stop_cm`, `clear_margin_cm`, `resume_delay_s`, `blind_near_cm`, `sector_deg`, `front_edge_cm`, `rear_edge_cm`, `scan_max_age_s`, `warn_after_s`, `scan_topic`(scan). `base_frame`(base_link)은 노드 파라미터만 있다.
- 2D 시뮬(`test/test_obstacle.py`, CI 포함): 시뮬 라이다 = 원형 물체 + 바깥 벽(안쪽 가벽·OMX·선반 없음), 라이다 yaw = π 장착, 노드와 같은 계산.
  - 구동 지연 0·0.15·0.2 s × 엔코더 0.95·1.0·1.05: 정지 → 3 s 뒤 재개, 멈춘 뒤 물체가 다가와도 0 그대로 대기, 재개 뒤 노드 감지 위치가 그대로다.
  - 실제 라이다 반사·잡음·벽 오감지는 흉내 내지 않았다.
- 한계(감지 범위):
  - 2D 라이다 스캔 평면(바닥에서 약 12.5 cm = URDF base_link 위 9.7 cm + base_link 높이 2.8 cm)보다 낮은 물체는 보이지 않는다.
  - 부채꼴 ±20°는 바퀴 축 기준이라 앞 끝 근처에서 몸체보다 좁다(앞 끝 +5 cm에서 옆 ±3.3 cm, 몸체 반폭 4.4~5.5 cm). 그래서 앞 끝에서 약 11 cm 안의 앞 모서리 쪽 물체는 놓칠 수 있다(계약대로 구현).
  - 대기 칸 P5(N16)는 창가 벽까지 약 34 cm다(layout, 몸체 앞 끝에서는 약 32 cm). 벽·OMX·선반이 25 cm 안에 들어오는지 monitor로 먼저 본다.

실로봇 시험(사용자 수행, 바퀴 띄움 → 바닥 저속, 정지 = `stop`·Ctrl+C):
1. `obstacle_mode:=monitor`로 평소 경로를 돌고, CSV `front_cm`·`rear_cm`로 벽·OMX·선반 오감지를 확인한다.
2. `stop`으로 직선 구간에 장애물을 두고 정지·3 s 뒤 재개·노드 세기를 확인한다.
3. 정지 중 물체를 다가오게 해도 로봇이 멈춘 채 기다리는지 본다.
4. 라인 유실 → 1 s 뒤 정지하는지 본다(유실 복구 없음).

## 토픽

| 토픽 | 형식 | 내용 |
| --- | --- | --- |
| `ir_sensor/range` | `std_msgs/UInt16MultiArray` | 입력. `pinky_sensor_adc` 약 20 Hz |
| `odom` | `nav_msgs/Odometry` | 입력. `pinky_bringup` 엔코더 |
| `scan` | `sensor_msgs/LaserScan` | 입력(`obstacle_mode` monitor·stop일 때만, sensor-data QoS). TF `base_link` ← scan frame 필요 |
| `line_nav/command` | `std_msgs/String` JSON | `{"cmd":"start","route":["N16","N14",...]}`(+ `start_cross`·`start_facing`) · `{"cmd":"stop"}` · `{"cmd":"load_done"}` · `{"cmd":"speed","base_speed"?,"rotate_speed"?}` · `{"cmd":"dock",...}` · `{"cmd":"pause"}` · `{"cmd":"resume"}` |
| `cmd_vel` | `geometry_msgs/Twist` | 출력. 주행 중일 때만, `dry_run` 이면 없음 |
| `line_nav/state` | `std_msgs/String` JSON | `state`·`reason`·`route`·`leg`·`traveled_cm`·`window_cm`·`events`·`ir`·`last_command`·`graph_version`·`dock`·`pause`·`pause_reason`·`obstacle`·`reverse`·`end` |

`state`: `idle` · `follow` · `pass` · `advance` · `rotate` · `loading` · `reverse` · `paused` · `arrived` · `handoff` · `stopped` · `error`.
`start_offset_cm`(선택, 0~50)은 출발 노드에서 센서까지 거리다. 없으면 첫 구간은 최소 거리 제한 없이 감지한다.

## 실행 (실로봇, 사용자 수행)

`line_follower`와 같이 띄우지 않는다(둘 다 `cmd_vel` 발행).

속도 명령 경로: launch로 띄우면 `line_nav → line_nav/cmd_vel → cmd_vel_guard → cmd_vel → pinky_bringup`이다.
- `pinky_bringup`은 cmd_vel 타임아웃이 없어 마지막 속도를 계속 유지한다. `cmd_vel_guard`(별도 프로세스)는 받은 명령을 넘기고, 움직이는 중에 0.3 s 넘게 명령이 끊기면(line_nav가 죽거나 멈춤) 0을 3번 보낸다. 코드 상한(0.15 m/s·1.5 rad/s)으로 한 번 더 자른다.
- guard 자체가 죽는 경우는 막지 못한다. 근본 해결은 구동 드라이버의 타임아웃이다.
- `cmd_guard:=false`면 예전처럼 line_nav가 `cmd_vel`에 바로 낸다. 테스트 콘솔 조종(teleop)·전체 정지는 지금도 `cmd_vel`에 바로 낸다. pinky_6422는 명령 앞에 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`를 붙인다.

```bash
# 빌드: roport_gz_world(노드 그래프), roport_pinky_line(IR 판정), roport_line_nav
colcon build --packages-select roport_gz_world roport_pinky_line roport_line_nav && source install/setup.bash
# 터미널 1 — 모터·odom
ros2 launch pinky_bringup bringup_robot.launch.xml
# 터미널 2 — IR 노드 + 경로 주행
ros2 launch roport_line_nav line_nav.launch.py robot:=pinky_d660
# 터미널 3 — 상태
ros2 topic echo --full-length /line_nav/state
# 터미널 4 — 명령
ros2 topic pub --once /line_nav/command std_msgs/msg/String "{data: '{\"cmd\": \"start\", \"route\": [\"N14\", \"N16\"]}'}"
ros2 topic pub --once /line_nav/command std_msgs/msg/String "{data: '{\"cmd\": \"load_done\"}'}"
ros2 topic pub --once /line_nav/command std_msgs/msg/String "{data: '{\"cmd\": \"stop\"}'}"
```

테스트 웹 콘솔(`pinky_test_console`)의 `mode:=test`(경로+IR)·`mode:=route` 화면에서도 경로 지정·시작·정지·상차 완료를 할 수 있다.

경로 상세 기록: 경로 하나마다 `~/roport_logs/route_<YYYYmmdd_HHMMSS>_<출발>-<도착>.csv`(IR 20 Hz 한 줄씩: state·구간·이동 거리·회전·IR 원시값·on_line·오차·cmd_vel·odom·이벤트)를 쓰고 최근 50개만 남긴다. 노드 파라미터 `log_dir`(빈 값이면 기록 안 함)·`log_keep`. 쓰기 오류가 나면 기록만 끄고 주행은 계속한다.

조정 인자(launch): `base_speed`, `approach_speed`, `approach_lead_cm`, `post_turn_slow_cm`, `kp`·`kd`·`gain_ref_speed`, `gate_min`·`gate_max`·`gate_early_cm`, `pass_full_speed`·`tee_both`·`tee_window_cm`·`kp_speed_scale`(노드 파라미터), `max_linear`, `max_angular`, `rotate_speed`, `reverse_speed`, `ir_ahead_cm`(실측 3), `turn_advance_cm`(4.2), `layout_file`, `dock_file`, `obstacle_mode`·`scan_topic`·장애물 값(위 절).

### 상태 표시 (LCD·LED, `status_indicator`)

`line_nav/state`·`battery/percent`를 구독만 하고 구동 명령은 보내지 않는다. LCD에 큰 영어 문구, 작은 둘째 줄(구간 `N19 -> N11`·오류 이유), 맨 아래 배터리(그림 + %, 40% 미만 노랑·20% 미만 빨강, 15 s 넘게 안 오면 `--`), LED에 색·점멸을 낸다. 상태 표는 `status_indicator.py`의 `STYLES` 한 곳에 있다(2026-11-07 사용자 요청: 파란 바탕 흰 글자, MOVING 파랑 점멸, DOCKING 주황 점멸, 나머지는 추천값).

| 상태 | 언제 | LCD 바탕 | LED |
|---|---|---|---|
| READY | 대기(idle), 상차 경로 끝 | 파랑 | 흰색 고정(밝기 25%) |
| MOVING | 노드 사이 이동·회전·통과 | 파랑 | 파랑 점멸 1 Hz |
| DOCKING | 상차 칸(P4·P7·N21·N22) 마지막 구간과 90° 회전 | 파랑 | 주황 점멸 1 Hz |
| DOCKED | 상차 칸 도착·상차 완료 대기(둘째 줄 `N11 waiting load`) | 파랑 | 주황 고정 |
| PARKING | 대기·충전 칸 마지막 구간, 180° 회전, 후진 | 파랑 | 초록 점멸 1 Hz |
| PARKED | 대기·충전 칸 도착 | 파랑 | 초록 고정 |
| HANDOFF | N12 Nav2 전환 | 파랑 | 보라 고정 |
| PAUSED | 수동 일시정지·장애물 정지·scan 끊김·앞 점 없음(둘째 줄 이유) | 파랑 | 노랑 고정 |
| STOPPED | 정지 명령 | 빨강 | 빨강 고정 |
| ERROR | 라인 놓침 등 오류 | 빨강 | 빨강 점멸 2.5 Hz |
| NO SIGNAL | `line_nav/state`가 `stale_sec`(2 s) 넘게 끊김 | 회색 | 끔 |

```bash
# 로봇: LED 서버(원본 pinky_led) — led_backend:=led 일 때
ros2 run pinky_led led_server
# 로봇: 상태 표시. LCD는 pinky_emotion 드라이버로 직접 그리므로 emotion_server와 같이 띄우지 않는다
ros2 run roport_line_nav status_indicator
# 램프 모듈(set_lamp, 점멸을 램프가 처리)을 쓸 때 / LCD 없이
ros2 run roport_line_nav status_indicator --ros-args -p led_backend:=lamp -p lcd:=false
```

- 파라미터: `led_backend`(`led`·`lamp`·`none`), `lcd`(true), `stale_sec`(2.0), `font_path`(DejaVuSans-Bold).
- LED 점멸은 `set_led` fill/clear를 바뀔 때만 부른다(1 Hz 점멸 = 초당 2번). 종료(Ctrl+C) 때 LED를 끄고 LCD를 검게 한다. 강제 종료(`kill -9`)에는 마지막 색이 남는다.
- LCD 방향·글자 크기는 노트북 그림(`render`)으로만 확인했다. 실제 LCD 방향은 로봇에서 확인한다.

## 빌드·테스트

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select roport_pinky_line roport_line_nav
source install/setup.bash
python3 -m pytest src/roport_line_nav/test -q
```

`test_executor.py`는 시뮬레이터로 Map 페이지 경로 예시 5개를 끝까지 달린다. 엔코더 ±7%와 테이프 위치 차이 최대 3.6 cm를 넣은 경우도 확인한다.
`Sim(cmd_delay_s=…)`로 구동 지연을 흉내 낸다(기본 0). 0.06 m/s + 지연 0.2 s에서 감속이 없으면 회전 중 선을 못 찾아
정지하고, 기본 감속이면 주요 경로 6개(P5→P4, P4→P7, P7→P5, H1→P7, P7→N12, 경유)를 완주하는 것을 확인한다.
노드 놓침·정지 명령·odom 없음도 확인한다. `test_side_rise.py`는 옆선 켜짐 확인(실로그 11-09 21:54 N02→N14 구간 재생 `test/data/`, 결함 주입 시뮬)을 확인한다. `test_executor.py`의 `test_line_gap_stops_after_lost_timeout`은 테이프 끊김(`Sim(gaps=…)`)에서 1 s 뒤 `line_lost` 정지를 확인한다. `test_dock.py`는 정밀 도킹 값(목표 자세·경유·범위·파일)과 교차선 출발을 같은 시뮬로 확인한다. `test_pause.py`는 일시정지·끝 기억을, `test_obstacle.py`는 라이다 TF 계산·장애물 정지·대기·scan 끊김·앞 점 없음·사각 의심을 확인한다. `test_node_ros.py`는 `ROS_DOMAIN_ID` 197과 전용 namespace를 써서 실제 로봇 `/cmd_vel`에 닿지 않는다.

## 한계

- 옆 선 감지는 「가운데 + 옆 채널」 패턴이라, 직선에서 라인이 가운데와 옆 센서 사이에 오면 같은 패턴이 나온다. 이동 거리 창(±30%, 이어지는 구간은 예상 − 5 cm부터) 밖에서는 무시하고, 창 안에서 잘못 감지하면 회전 중 선을 못 찾아 정지한다.
- 90° 회전·후진 거리는 odom(엔코더)만으로 잰다. 바퀴 미끄러짐은 보정하지 않는다.
- 실측과 테이프 위치가 다르면 예상 거리 창이 틀어진다. 배치 yaml의 노드 좌표를 실측으로 맞춰 둔다.
