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
  - 11-07 17:55 N14→N02, 16:57 N14→N15: 가장자리 run(raw 약 1000~1200)이 꺼지지 않은 채 진짜 옆선(raw 300~500)으로 이어짐 → 새 규칙은 오감지하지 않지만 **진짜 노드도 감지하지 못한다** → 노드를 지나 `node_missed`나 `gap into … window`로 정지한다(오감지 뒤 회전 실패 대신 안전 정지).
  - 진짜 감지 중 새 규칙에서 놓친 것 0건. 진짜 옆선 run은 창 시작보다 0.5 cm 이상 뒤(98구간, 최소 0.5 cm: 11-05 N19→N09).
  - 재생의 한계: 명령이 기록과 달라지면 이후 궤적도 달라지므로 기록된 직진 구간(추종 + 회전 전 전진)까지만 판정을 비교한다. 그 뒤의 결말(위 두 건의 정지)은 추론이다. 11-04~11-05 기록은 당시 그래프(v13), 그 뒤는 v14로 재생했고, 감지 창은 지금 코드(gate_early·tee_both 포함)로 계산했다.
- T자(`tee_both`)에는 기본으로 쓰지 않는다(`side_rise_tee` 꺼짐). T자는 반대쪽 IR이 `tee_window_cm` 안에 함께 켜져야 해서 한쪽 가장자리 타기는 이미 가로선에서 맞게 감지되고, 켜짐 확인을 더하면 그 경우가 놓침이 된다. 실로그의 진짜 T자 37건은 run이 창 시작보다 4.3 cm 이상 뒤라 켜도 결과는 같았다.
- 감지 창 안 GAP(라인 유실 복구)에서도 같은 run 추적을 쓴다. 유실 동안 옆 IR이 꺼져 있으므로, 재획득한 옆선이 창 안이면 켜짐으로 인정된다.
- 약점: 노드가 odom으로 예상보다 5 cm 넘게 일찍 오면(엔코더 과소) 진짜 옆선 run이 창 시작 전에 시작해 놓칠 수 있다 → `node_missed` 정지. 2D 시뮬 엔코더 −7%(N02→N10 88.9 cm)에서 일어남. 실로그의 진짜 감지는 예상 −3.4 cm 안이었다.
- 2D 시뮬은 PD가 실로봇보다 빨리 선에 맞춰져 가장자리 타기가 재현되지 않는다. 그래서 실행부가 받는 왼쪽 IR 판정만 8~16 cm 동안 켜짐으로 덮어써(결함 주입) 시험했다. 결과: 0.03·0.06·0.08 m/s × 지연 0·0.15·0.2 s에서 예전 규칙은 9/9 오감지 → `line_not_found`(실로봇과 같음), 새 규칙은 9/9 완주(주입 없을 때와 같은 위치에서 감지).
- 파라미터(노드): `side_rise_only`(true), `side_rise_off_samples`(2), `side_rise_tee`(false).

## 라인 유실 복구(FFT1-163, 실로봇 미검증)

테이프가 3~5 cm 끊기거나 벗겨진 곳을 건너기 위한 동작이다. 추종 중에만 쓴다. 노드 근처에서는 직진만 하며 그 노드의 평소 판정으로만 노드를 찾고, 못 찾으면 정지한다.

| 단계 | 내용 |
| --- | --- |
| GAP | 직선 추종 중 IR 3개가 모두 선을 놓쳤고 이동 거리가 구간 시작 `suppress_cm`(4 cm) 뒤이면, 놓치기 전 `gap_heading_window_cm`(4 cm) 동안의 선 방향을 유지하며 `approach_speed` 이하로 직진(최대 `gap_max_cm` 8 cm). **IR 하나라도** `gap_confirm`(2) 샘플 연속 선 위면 추종으로 돌아감(이전 오차 지움, 옆 IR이면 추종 PD가 그쪽으로 조향). 이 기능 전에도 IR 하나라도 선 위면 추종했고 추종의 노드 판정은 그대로라 판정 완화가 아니다. 넘으면 정지 `line_lost: gap N cm exceeded 8 cm`. 시간 상한 = 2 + 2·`gap_max_cm`/`min_speed`(기본 약 12.7 s, 도킹 이동 timeout과 같은 꼴) → `line_lost: gap timeout`. odom 이동이 0.1 cm 미만인 채 `lost_timeout`(1 s)이 지나면(odom은 오는데 값이 그대로 등) → `line_lost: gap no progress`. 이벤트 `gap`·`gap_recovered` |
| 선 방향 | 선 위 추종 샘플마다 선 위치(가운데 IR 위치 − 오차 × `ir_spacing_cm`, 옆 방향)를 기억해 앞·뒤 절반 평균을 이은 방향. 회전 직후처럼 로봇이 선과 10~16° 비스듬히 추종 중이어도 선 방향을 쓴다(로봇 yaw 평균은 2D 시뮬 N02→N10 앞쪽 5 cm 끊김에서 9번 중 2번 실패). 기억이 2 cm 미만이면 odom 방향 평균 |
| SEARCH(**기본 끔**, `search_deg` 0) | `search_deg` > 0일 때만. `gap_max_cm` 안에 못 찾으면 멈춤 확인(정밀 도킹 이동과 같은 규칙) → 놓친 지점까지 후진 → 제자리 좌 `search_deg`(예: 20°) → 우 2배 → 좌(원래 방향) 순서로 돌며 가운데 IR이 선 위·`align_error` 안이면 그 자리에서 추종(놓친 지점 이동 거리부터). 못 찾으면 `line_lost: gap/search failed on A→B`. 구간마다 한 번만(두 번째 GAP이 넘치면 바로 같은 오류). 이벤트 `search`·`search_found`·`search_failed`. 2D 시뮬에서 SEARCH로 선을 다시 잡아 끊김을 넘은 경우가 0건이다(놓친 지점에서 돌아서 가운데 IR이 옆으로 약 ±1 cm만 움직임 — 대부분 끊기기 전 선 끝을 잡고 두 번째 GAP에서 정지). 그래서 기본 끔이고, 켜도 안전 정지 위주다 |
| 노드 근처(감지 창 안) | 놓친 곳이 이미 감지 창 안이거나 GAP이 창에 들어가면, 같은 방향·속도로 직진하며 **그 노드의 평소 판정**(가로선 = IR 3개, 옆선 = 가운데 + 옆, T자 = `tee_both`·`tee_window_cm`, `node_confirm` 연속)으로만 노드를 찾는다. 노드 테이프에 닿아야만 감지(판정 완화 없음) → 평소처럼 도착 동작. 노드가 아닌데 IR이 선을 보면 창 밖과 같은 규칙으로 추종에 돌아간다(추종의 노드 판정 그대로). 감지 없이 `max_ok`나 놓친 곳부터 `gap_max_cm`를 넘으면 `line_lost: gap into <노드> window, node not found`로 정지. 창 안에서는 SEARCH(후진·회전)를 하지 않는다(노드 근처에서 다른 선을 잡을 수 있음). 이벤트 `gap_window`·`gap_node` |
| 그대로인 곳 | 구간 시작 `suppress_cm` 안 유실 → 예전 동작(0 명령으로 `lost_timeout` 1 s 뒤 `line_lost on A→B`). 통과·회전 전 전진·회전·후진·도킹 이동은 예전처럼 유실을 보지 않고 거리·각도로 끝냄(바뀐 것 없음). 예상 거리(`max_ok`)를 넘은 뒤 처음 유실 → 예전처럼 `node_missed`. 출발 위치를 몰라 감지 창이 0부터인 첫 구간은 처음부터 「창 안」 규칙 |

- 남은 위험(실로봇 미검증):
  - 창 전 GAP 중에는 노드 판정을 하지 않고 IR이 선을 보면 복귀하므로, 그래프에 없는 가로지르는 테이프(다른 시험용 테이프 등)가 있으면 그 선을 「다시 잡은 선」으로 보고 추종할 수 있다. 시연 바닥에 그래프 밖 테이프가 없게 한다.
  - 옆으로 밀려 선을 놓쳤으면 GAP은 선과 나란히 가므로 옆 IR이 다시 보지 못할 수 있다 → 최대 8 cm 직진 뒤 정지(예전은 그 자리 0 명령 뒤 1 s에 정지).
  - GAP은 odom 거리로 끝을 정하므로 시간 상한과 진행 없음 검사를 둔다(위). 강제 종료(`kill -9`)에는 0을 못 보내는 것은 그대로.
- 상차·대기 칸 노드 진입로가 노드 앞 약 6.5 cm까지 끊기면 가로선은 감지하지만, 도착 뒤 진입선 찾기 회전(상차 2번째 회전·대기 칸 180°)이 노드에서 약 7 cm 지점 테이프를 봐야 해서 `line_not_found`로 정지할 수 있다. 정차 노드 앞 8 cm 안 테이프는 끊기지 않게 관리한다.
- 상태(`state`)는 GAP 중 `follow` 그대로, SEARCH 중 `advance`(후진)·`rotate`(찾기)다. 테스트 콘솔·관제 trial_driver의 「주행 중」 목록을 바꾸지 않으려는 것. 무엇을 하는지는 `line_nav/state`의 `recovery`(`{"mode":"gap","lost_at_cm","gap_cm","in_window"}`·`{"mode":"search","lost_at_cm"}`, 평소 `null`)와 이벤트로 본다. 화면 추정(`pose_est`)은 GAP 중 직진, SEARCH 중 놓친 지점.
- 파라미터(노드·launch): `gap_max_cm`(8, 0 = 끔 = 예전 동작), `gap_heading_window_cm`(4), `gap_confirm`(2, 노드 파라미터만), `search_deg`(0 = 끔, 기본), `ir_spacing_cm`(2.0, Pinky Pro HW Spec 약 20 mm, **실측 아님**).
- 끊김이 없으면 예전과 같다(복구 켬·끔 궤적 비교 테스트).
- 2D 시뮬(0.03·0.06·0.08 m/s × 구동 지연 0·0.15·0.2 s, 위치 3곳 N05→N07 가운데·N16→N14·N02→N10 회전 직후): 2·3·5 cm 끊김 81번 모두 완주. 8 cm는 긴 직선 9/9 복구, 회전 직후 8/9 복구(1번 GAP 초과 정지), 짧은 구간 N16→N14는 6/9 창 안 복구·3/9 정지. 10 cm 이상은 모두 정지(GAP 초과 또는 창 안 노드 없음). 끊긴 곳에서 실제 방향 ±3~5° 틀어짐(odom은 모름)도 3·5 cm 완주.
- 노드 진입로 끊김(노드 중심 앞 1.5 cm까지, 노드 테이프 유지, 9가지 속도·지연): 3 cm는 N11·N01(상차)·N16(대기 칸)·N10(모서리)·N14(T자)·N15(통과) 모두 9/9 노드 감지·완주. 5 cm는 N10·N14 9/9, N15 8/9(1번은 통과 뒤 짧은 다음 구간 끝 N05 회전에서 정지), N11·N01·N16은 노드 감지 9/9·완주 3/9(6/9는 도착 뒤 진입선 찾기 회전에서 정지, 위 항목). N10 모서리·N14 T자 테이프를 없애면 노드를 건너뛰거나 돌지 않고 정지.
- 실로봇 시험(제안): 바퀴 띄움 → 바닥 저속(0.03 m/s) 순서, 정지는 `stop` 명령·Ctrl+C. 카펫과 같은 색 종이로 테이프를 가린다. 예: N05→N07 직선 가운데(N05에서 40 cm)를 3 cm → 5 cm, N16→N14(P5 출발 6 cm 뒤) 3 cm. 노드 진입로(노드 앞 1.5~4.5 cm) 3 cm도 시험하되, 상차·대기 칸 노드 앞 8 cm 안은 5 cm 이상 가리지 않는다. route CSV에서 `gap`·`gap_recovered` 이벤트와 복구 거리를 확인한다.

오류(`error`)로 멈추는 경우: 예상 거리를 지나도 노드 없음(`node_missed`), 직선 추종 중 라인 놓침 1초 이상(복구를 켜면 구간 시작 `suppress_cm` 안에서만, `gap_max_cm` 0이면 어디서나), 라인 유실 복구 실패(`line_lost: gap … exceeded`·`gap into <노드> window, node not found`·`gap timeout`·`gap no progress`, SEARCH를 켰으면 `gap/search failed`), 회전 중 선을 못 찾음,
후진 초과, IR 끊김 0.3초(`no_ir`), odom 끊김 0.5초(`no_odom`). 정지·오류 때 0 속도를 한 번 발행하고 그 뒤로는 발행하지 않는다.

경로가 끝나면 로봇은 다음 출발 방향을 보고 선다. 상차 노드는 진입선 쪽, 대기·충전 칸은 칸 바깥쪽이다.
그래서 다음 경로를 그 노드에서 바로 시작할 수 있다.

이어서 출발: 실행부는 마지막으로 끝난 노드·방향·센서 위치를 기억한다.
- 끝난 노드에서 다시 출발하면 그 센서 위치로 첫 구간 거리 창을 정한다(상차 노드 7.2 cm, 대기 칸 1.2 cm).
- 첫 구간이 로봇이 보는 방향과 반대이면(예: N12 handoff 뒤 `N12 → N08 …`) 먼저 반시계로 반 바퀴(172~210°) 돌아 진입선을 찾는다(이벤트 `pre_rotate`).
- 정지·오류 뒤에는 기억을 지운다. 사람이 로봇을 옮긴 경우 첫 구간 방향으로 세우고 정지한 뒤 시작한다.
- 기억이 없으면 가로선 노드 출발은 센서가 가로선 위(0 cm)에 있다고 보고 거리 창을 쓴다. 그 밖의 노드는 첫 구간 최소 거리 제한이 없다.

## 일시정지·이어 가기·라인 후진(FFT1-172, 실로봇 미검증)

우선순위가 높은 로봇에게 길을 비켜 주려고 지나온 노드까지 라인을 따라 뒤로 간다(예: N19 근처 → N23). 관제 MQTT 연동은 아직 없다(테스트 콘솔·`line_nav/command`만).

| 명령 | 동작 |
| --- | --- |
| `{"cmd":"pause"}` | 0 속도, 상태 `paused`. 지금 단계·구간·이동 거리·감지 창·선 방향 기억을 그대로 둔다. 시간 제한 없음. odom이 멈추면(\|v\| < 0.003 m/s, \|ω\| < 0.03 rad/s가 0.3 s) `pause.settled` = true·이벤트 `paused_settled`. 끝난 경로(`idle`·`arrived`…)에서는 거부 |
| `{"cmd":"resume"}` | `paused`에서만. 멈춘 단계를 그대로 이어 감(감지 창·누적 거리 그대로, 시간 초과 기준은 멈춘 시간만큼 밀림) |
| `{"cmd":"back","to":"N23"}` | `to` = 이번 경로에서 지나온 노드(마지막 경유 정차 뒤). 받는 때: `paused`(정지 확인 뒤)·`loading`(정지 확인 뒤)·가로선 정지(`stop`)로 끝난 `arrived`(정지 확인 뒤, 끝난 뒤 `stop`을 받지 않았고 odom이 감지 자세에서 2 cm·5° 안). 도착하면 `arrived`, `route` = 지나온 앞부분(`route[-1]` = `to`, 경유 정차 노드까지 되감았으면 끝 구간 `action` = `stop`). 지금 받을 수 있는 노드는 상태의 `back_targets` |
| `{"cmd":"stop"}` | 지금처럼 완전 정지(`paused`·후진 중에도) |
| `{"cmd":"speed","reverse_speed":0.03}` | 라인 후진 속도 0.01~0.03 m/s(그리고 `max_linear` 이하, 기본 0.02). 주차 후진 `reverse_speed`(P5·P6) 설정과는 다른 값이다 |

`back` 거절(움직이지 않음, `last_command.ok` false): 멈춤 확인 전, 회전 중·라인 유실 복구(GAP·SEARCH) 중·도킹 이동 중·주차 후진 중 멈춤, 출발 회전(교차선 출발·되돌아 출발) 중, `to`가 지나온 경로에 없음(앞쪽 노드·경로 밖·경유 상차를 거꾸로 지나야 함), 상차·주차 뒤 되돌아 선 `arrived`, `handoff`(N12, Nav2 소유), 끝난 뒤 `stop`(위치 기억을 지움), 끝난 뒤 odom이 감지 자세에서 2 cm 또는 5° 넘게 바뀜(손·Nav2로 옮겨졌을 수 있음), 이미 후진했음(새 경로를 시작한 뒤 다시), 선 방향을 모름(구간을 2 cm도 못 따라감).

되감는 순서(구간 단위):
1. 시작: 직선 추종·회전 전 전진·직진 통과 중이면 그 자리. 상차 대기면 도킹 이동을 먼저 되돌림(−b → 시계 r° → −a, 기본 값이면 시계 90°만). 상차 완료(`load_done`)는 보내지 않는다. OMX가 움직이지 않는다는 확인은 관제 몫이다.
2. 선 방향(odom)이 2° 넘게 다르면 제자리로 맞춤(`rv_align`).
3. 라인 후진(상태 피드백, 아래) → 이 구간 목표 노드를 **전진과 같은 판정**(가로선 3채널·옆선 + 켜짐 확인·T자 `tee_both`·`node_confirm`)으로 찾으면 즉시 정지. 감지 창 = 예상 거리 E 기준 기존 규칙(최소 max(0.7E − 3, E − 5), 최대 1.3E + 3, 사람이 놓은 첫 구간은 비율만), 처음 `suppress_cm` 판정 안 함. 직진 노드는 멈추지 않고 지나간다.
4. 그 노드에서 회전해 왔으면: 축을 노드 중심으로(멈춘 뒤 `ir_ahead − 테이프 반폭` 전진) → 그 회전을 반대로(직진 방향 선이 있으면 선 찾기 60~120°) → 2번 → 다음 구간 후진(예상 = 구간 길이 + `ir_ahead` − 반폭). 직진 방향 선이 없는 노드(모서리 N02, 분기에서 올라와 돈 T자 N23·N18)는 2번만으로 돌리고, 센서가 테이프 끝 너머 약 1.8 cm에서 시작하므로 그 구간 처음 `ir_ahead − 반폭 + 1.5 cm`(3.3 cm) 안에서는 방향만 유지하며 선을 다시 잡는다(놓침을 세지 않음, 못 잡으면 `reverse_lost: line not reacquired`). `reverse_lost_cm` 기준은 그대로다.
5. 출발 노드에서 센서가 `reverse_short_cm`(6 cm) 안이면 판정 대신 odom 거리만큼 이동(목표: 마지막이면 센서가 노드 중심, 회전을 되돌릴 노드면 축이 노드 중심).

끝 자세 = 그 노드에 같은 방향으로 전진 도착해 감지한 자세(센서가 노드 테이프 위, 축은 3 cm 뒤). 다음 출발은 이어서 출발 규칙(센서 위치 = 테이프 반폭, `gate_early_cm` 적용)으로 그대로 간다(예: `N23 → N19 → N11`). 첫 구간이 좌·우 회전이면(예: `N23 → N21`) 회전 전에 축을 노드 중심으로 전진 보정한다(감지 자세 + `ir_ahead − 반폭`, 멈춘 뒤 odom 투영, 이벤트 `pre_center`, 첫 구간 센서 위치 = `ir_ahead`). 보정이 없으면 축이 중심 약 2.1 cm 뒤에서 돌아 2D 시뮬 9/9 `line_not_found`였다. 후진 도착 뒤 odom이 2 cm 또는 5° 넘게 바뀌었거나(조종 등) 보정 이동이 `ir_ahead − 반폭 + 1 cm`(2.8 cm)를 넘으면 `start`를 거절하고(움직이지 않음) 끝 기억을 지운다. 그 뒤 같은 `start`는 기억 없는 출발(보정 없음)이다. 반 바퀴 출발은 예전 규칙 그대로. 상태 `reverse`는 P5·P6 주차 후진과 같은 이름이고, 라인 후진이면 `line_nav/state`의 `reverse`가 null이 아니다.

이어 출발 때 지나온 길 잇기(실로봇 시험 뒤 후속): 라인 후진으로 끝난 노드(끝 기억)에서 `start`의 `route[0]`이 그 노드이면 직전 경로의 지나온 길(후진으로 줄인 경로, 앞서 이어 붙인 길 포함)을 이번 경로 앞에 붙인다. 그래서 새 경로 첫 구간에서 멈춘 뒤 `back`이 직전 경로의 노드까지 되감는다(예: P5 → N01 중 back N18 → `["N18","N01"]` 출발 → N18→N01 중 pause → `back_targets` = `["N18","N10","N02","N14","N16"]`, back N10 = 한 경로에서 바로 back N10 한 것과 같은 동작). 되감기 규칙·판정 기준·거절 조건은 그대로다(구간 단위, 이음 노드의 회전도 되돌림, 경유 정차 노드를 넘지 않음). 이음 노드의 회전 = 직전 경로가 그 노드로 들어온 방향 → 새 첫 구간 방향(직진이면 멈추지 않고 지나감).
- 붙이지 않는 때(이번 경로만, 예전과 같음): 끝난 뒤 `stop`(기억 지움), 끝난 자세에서 odom이 2 cm 또는 5° 넘게 바뀜·odom 끊김, 끝 노드가 가로선 정차 노드(전진 도착·경유/대기 칸까지 후진), `route[0]`이 끝 노드가 아님, U턴 출발(직전 진입 방향의 반대), `start_cross`·`start_offset_cm`(사람이 놓은 출발로 봄).
- 끝난 뒤 odom이 2 cm 또는 5° 넘게 바뀌었거나 odom을 모르면(`end.moved`가 true·null) 같은 노드 직진 출발이라도 끝 기억을 버리고 손으로 놓은 출발과 같게 간다(첫 구간 창 최소 0·`gate_early` 없음, 이어 붙이기·`pre_center` 없음, 이벤트 `end_dropped`, 거절하지 않음). 회전 출발 거절(위 문단)은 그대로.
- 상태의 `route`·`index`는 `back`을 시작하기 전까지 명령한 경로 그대로다(관제 trial_driver 비교 유지). `back`을 시작하면 붙인 앞부분까지 포함해 보고하고, 도착 뒤 `route`는 지나온 길 전체(`route[-1]` = `to`). 이벤트 `chain`(붙인 앞 경로).
- 상태 `end`: 끝 기억이 있으면 `{"node": "N18", "facing": "S", "moved": false, "passed": ["N16","N14","N02","N10"]}`, 없으면(주행 중·`stop` 뒤·오류 뒤) null. `facing` = E·N·W·S, `moved` = 끝난 자세에서 2 cm 또는 5° 넘게 바뀌었는지(odom 모르면 null), `passed` = 이 노드에서 이어 출발하면 붙는 지나온 노드(경로 순서, 마지막 경유 정차 노드까지 — back이 넘지 않는 곳), 붙일 수 없으면 `[]`. 전진으로 끝난 상차·주차·handoff도 `end`는 보고하지만 `passed`는 항상 `[]`.

후진 제어: IR이 바퀴 축 앞 `ir_ahead_cm`(3 cm)에 있어 전진 PD를 거꾸로 쓰면 회전 → IR 오차 전달함수 (d·s + v)/s²의 영점이 후진(v < 0)에서 우반면(+|v|/d, 0.02 m/s에서 +0.67 rad/s)으로 가 불안정하다. 그래서 방향(odom)과 옆 오차(IR)를 나눠 되먹임한다.
- θ̂ = odom yaw − ψ(선 방향), ŷ = e_cm − d·sin θ̂, e_cm = IR 오차 × `ir_spacing_cm`(+ = 선이 센서 오른쪽 = 축이 선 왼쪽). ω = +k_y·ŷ − k_θ·θ̂, k_y = ωₙ²/|v|(rad/s per cm, v는 cm/s), k_θ = 2ζωₙ, `max_angular`로 자름.
- 유도: 선 좌표에서 ẏ = v·sin θ, θ̇ = ω. v < 0이면 θ̈ + k_θ·θ̇ + k_y·|v|·θ = 0 → 계수가 양수라 안정. 기본 ωₙ 0.8 rad/s·ζ 0.8 → 0.02 m/s에서 k_y 0.32, k_θ 1.28.
- ψ = 그 구간을 전진할 때 `suppress_cm` 뒤 ~ 감지 창 시작 전 선 위 샘플(라인 유실 복구와 같은 선 위치 점, 최근 30 cm)의 앞·뒤 절반 평균 방향. 없으면 지도 방향 + 다른 구간에서 구한 odom 차이, 그것도 없으면 지금 구간의 GAP 방향. 12 cm 창은 2D 시뮬에서 IR 불감대 안 느린 흐름 때문에 1.6° 틀려 30 cm로 늘렸다.
- 방향만 유지(ŷ 안 씀): 처음 `suppress_cm`, 지나치는 노드 옆선 ±(반폭 + 2 cm), 목적 노드 앞 max(창 시작, E − 5 cm) 뒤·목적 노드 테이프를 밟은 뒤(옆선이 오차를 치우쳐 마지막에 차체를 돌리는 것 방지).

실패(0 속도 한 번 → `error`): `reverse_lost`(IR 3개 모두 놓침이 `reverse_lost_cm` 2 cm 이상, 또는 놓친 채 3 s. GAP·SEARCH 안 함), `reverse_node_missed`(창 최대까지 목적 노드 없음), `reverse_heading`(\|θ̂\| > `reverse_max_heading_deg` 15°), `reverse_timeout`(2 + 2·창 최대/1 cm/s 초), `reverse_move` 초과·시간 초과(짧은 이동), `line_not_found`(회전 되돌리기), `no_odom`·`no_ir`(기존과 같음).

상태 보고(`line_nav/state`):
```json
{"state": "paused", "pause": {"from": "follow", "settled": true}, "reverse": null, "...": "..."}
{"state": "reverse", "pause": null,
 "reverse": {"to": "N23", "phase": "reverse", "from": "follow", "leg": ["N23", "N19"],
             "expected_cm": 27.8, "done_cm": 12.4, "y_cm": 0.1, "theta_deg": -0.4,
             "window_cm": [22.8, 39.1]}}
{"state": "arrived", "reason": "reverse done at N23", "route": ["N07", "N23"],
 "leg": {"from": "N07", "to": "N23", "...": "..."}, "reverse": {"to": "N23", "phase": "done", "...": null}}
```
- `reverse.phase`: `start` → (`undo_dock`) → `rv_align`·`reverse`·`rv_center`·`rv_unturn`·`rv_short` → `done`. 회전·이동 단계의 `state`는 `rotate`·`advance`(기존 이름).
- `back_targets`: 지금 `back`으로 받을 노드 목록(최근 지난 순, 예 `["N23", "N07"]`), 받을 수 없으면 `[]`(주행 중·정지 확인 전·후진 중·후진 뒤·handoff·상차/주차 뒤). 테스트 콘솔 후보는 이 값을 쓴다.
- 이벤트: `pause`·`paused_settled`·`resume`·`back`·`reverse`(구간 시작: 예상·창)·`reverse_short`·`reverse_node`(판정)·`reverse_arrived`·`pre_center`(후진 끝에서 회전 출발 전 축 맞춤)·`chain`(이어 출발 때 붙인 지나온 길), 실패 `reverse_lost`·`reverse_node_missed`·`reverse_heading`·`reverse_timeout`. 후진 중에는 `node` 이벤트를 내지 않는다(관제 trial_driver가 `node`로 경로 진행을 셈).
- route CSV에 `rv_y_cm`·`rv_theta_deg` 칸(라인 후진 중 ŷ·θ̂)을 맨 끝에 더했다. LCD는 `paused` = PAUSED(노랑 고정, 추천값), 라인 후진 = MOVING(`back -> N23`).
- 관제 trial_driver(`roport_pinky`, 바꾸지 않음)의 「주행 중」 목록에 `paused`가 없다: 멈춤 동안 관측값 `line_enabled`·`drive_owner`가 꺼짐으로 보이고, 그때 route 명령이 오면 line_nav가 `busy`로 거부해 `line_nav_rejected`가 된다. 시험 중(trial 명령 실행 중) back으로 경로가 끝나면 `route`가 바뀌어 `line_nav_no_response`로 실패 처리된다. 관제 연동 때 협의한다.

**안전: 뒤쪽 감지가 없다.** line_nav는 라이다·초음파를 쓰지 않는다. 관제(지금은 시험자)가 후진 경로(예: N19 → N23)와 목적 노드가 비었음을 확인·예약한 뒤에만 `back`을 보낸다(물류 2호기가 N07 → N23으로 오고 있으면 부딪친다). 라이다 뒤쪽 정지는 FFT1-173에서 따로 한다. 강제 종료(`kill -9`)에는 0을 보낼 수 없는 것은 그대로다.

파라미터(노드, launch는 `reverse_follow_speed`·`reverse_wn`·`reverse_zeta`): `reverse_follow_speed`(0.02, 0.01~min(0.03, `max_linear`)), `reverse_wn`(0.8, 0.2~2.0), `reverse_zeta`(0.8, 0.3~1.5), `reverse_lost_cm`(2, 0.5~5), `reverse_max_heading_deg`(15, 5~30), `reverse_short_cm`(6, 0~15).

2D 시뮬(`test/test_reverse.py`, CI 포함): 경로 N07(교차선, 동쪽) → N23 → N19 → N11, 0.02·0.03 m/s × 구동 지연 0·0.15·0.2 s × 엔코더 0.95·1.0·1.05 × 시작 상황 6가지 = 108회 모두 N23 도착 → 이어서 `N23 → N19 → N11` 완주.

| 시작 상황 | 결과 | 센서 − N23 (cm) | 축 옆 (cm) | 방향 (°) | N23–N19 후진 중 최대 옆 (cm) |
| --- | --- | --- | --- | --- | --- |
| (가) N23→N19 25 cm에서 pause | 18/18 | +0.68 ~ +0.88 | −0.62 ~ −0.10 | −0.5 ~ +1.9 | 0.62 |
| (가) N19 감지 직후(회전 전 전진 중) | 18/18 | +0.61 ~ +0.93 | −0.59 ~ −0.14 | 0.0 ~ +1.2 | 0.59 |
| (가′) N23 통과 직후 1 cm | 18/18 | +0.23 ~ +0.58 | −0.58 ~ −0.10 | −0.6 ~ +0.9 | (odom 이동) |
| (나) N19 우회전 끝 0.5 cm | 18/18 | +0.64 ~ +0.93 | −0.58 ~ +0.09 | 0.0 ~ +1.3 | 0.58 |
| (다) N19→N11 15 cm | 18/18 | +0.61 ~ +0.94 | −0.59 ~ +0.10 | 0.0 ~ +1.2 | 0.59 |
| (다′) N11 상차 대기 | 18/18 | +0.62 ~ +0.92 | −0.59 ~ +0.17 | 0.0 ~ +1.2 | 0.59 |
| 참고: 전진으로 N23 감지 | 18 | −0.73 ~ −0.66 | −0.04 ~ 0.00 | −4.7 ~ −0.6 | − |

센서가 N23 테이프(반폭 1.2 cm) 위에 서는 것은 전진 감지와 같다(전진은 서쪽 가장자리, 후진은 동쪽 가장자리에서 감지). 같은 시작에서 흐트러짐(옆 −0.8~0.6 cm, 방향 −5~4°)을 넣으면 상태 피드백은 N23에 서고, 전진 PD 부호만 바꾼 후진은 15°를 넘어 정지한다(`reverse_heading`, 흐트러짐 없음 24.4 cm·있음 0.8~1.2 cm에서). 단순 운동학 모델에서도 전진 PD 그대로·부호만 바꾼 것은 둘 다 선 밖(> 3 cm)으로 나가고 상태 피드백은 0.01~0.03 m/s에서 수렴한다. 2D 시뮬의 PD 추종은 실로봇보다 빨리 선에 맞춰지므로 실로봇 시험이 필요하다.

실로봇 시험(제안, 사용자 수행): 바퀴 띄움 → 바닥 0.02 m/s 순서, 정지는 `stop`·Ctrl+C. N23→N19 가운데에서 pause → back N23 (가), N19 회전 직후 (나), N19→N11 중간 (다), N11 상차 대기 (다′). route CSV의 `rv_y_cm`·`rv_theta_deg`, 끝 자세(센서가 N23 테이프 위인지), 이어서 `N23 → N19 → N11` 완주를 본다. 후진 경로에 사람·로봇이 없게 한다.

## 토픽

| 토픽 | 형식 | 내용 |
| --- | --- | --- |
| `ir_sensor/range` | `std_msgs/UInt16MultiArray` | 입력. `pinky_sensor_adc` 약 20 Hz |
| `odom` | `nav_msgs/Odometry` | 입력. `pinky_bringup` 엔코더 |
| `line_nav/command` | `std_msgs/String` JSON | `{"cmd":"start","route":["N16","N14",...]}`(+ `start_cross`·`start_facing`) · `{"cmd":"stop"}` · `{"cmd":"load_done"}` · `{"cmd":"speed","base_speed"?,"rotate_speed"?,"reverse_speed"?}` · `{"cmd":"dock",...}` · `{"cmd":"pause"}` · `{"cmd":"resume"}` · `{"cmd":"back","to":"N23"}` |
| `cmd_vel` | `geometry_msgs/Twist` | 출력. 주행 중일 때만, `dry_run` 이면 없음 |
| `line_nav/state` | `std_msgs/String` JSON | `state`·`reason`·`route`·`leg`·`traveled_cm`·`window_cm`·`events`·`ir`·`last_command`·`graph_version`·`dock`·`pause`·`reverse`·`back_targets`·`end` |

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

조정 인자(launch): `base_speed`, `approach_speed`, `approach_lead_cm`, `post_turn_slow_cm`, `kp`·`kd`·`gain_ref_speed`, `gate_min`·`gate_max`·`gate_early_cm`, `pass_full_speed`·`tee_both`·`tee_window_cm`·`kp_speed_scale`(노드 파라미터), `max_linear`, `max_angular`, `rotate_speed`, `reverse_speed`, `ir_ahead_cm`(실측 3), `turn_advance_cm`(4.2), `layout_file`, `dock_file`, `gap_max_cm`·`gap_heading_window_cm`·`search_deg`·`ir_spacing_cm`(라인 유실 복구), `gap_confirm`(노드 파라미터), `reverse_follow_speed`·`reverse_wn`·`reverse_zeta`(라인 후진, 나머지는 노드 파라미터).

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
노드 놓침·정지 명령·odom 없음도 확인한다. `test_side_rise.py`는 옆선 켜짐 확인(실로그 11-09 21:54 N02→N14 구간 재생 `test/data/`, 결함 주입 시뮬)을 확인한다. `test_gap.py`는 테이프 끊김(`Sim(gaps=…)`)에서 GAP·SEARCH·노드 근처 정지를 확인한다. `test_dock.py`는 정밀 도킹 값(목표 자세·경유·범위·파일)과 교차선 출발을 같은 시뮬로 확인한다. `test_reverse.py`는 일시정지·라인 후진(부호·안정성, 시작 상황 행렬, 거절·실패)을 확인한다. `test_node_ros.py`는 `ROS_DOMAIN_ID` 197과 전용 namespace를 써서 실제 로봇 `/cmd_vel`에 닿지 않는다.

## 한계

- 옆 선 감지는 「가운데 + 옆 채널」 패턴이라, 직선에서 라인이 가운데와 옆 센서 사이에 오면 같은 패턴이 나온다. 이동 거리 창(±30%, 이어지는 구간은 예상 − 5 cm부터) 밖에서는 무시하고, 창 안에서 잘못 감지하면 회전 중 선을 못 찾아 정지한다.
- 90° 회전·후진 거리는 odom(엔코더)만으로 잰다. 바퀴 미끄러짐은 보정하지 않는다.
- 실측과 테이프 위치가 다르면 예상 거리 창이 틀어진다. 배치 yaml의 노드 좌표를 실측으로 맞춰 둔다.
