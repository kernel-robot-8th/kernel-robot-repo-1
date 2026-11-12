"""경로 주행 상태 기계(FFT1-117). 순수 Python, ROS 없음.

입력: IR 판정(roport_pinky_line.line_sensor.LineReading, 약 20 Hz), odom 자세(cm, rad), 시각(s).
출력: 속도 명령(선속도 m/s, 각속도 rad/s, ROS 기준 반시계 +) 또는 None(발행 안 함).

노드 감지 = IR 패턴 + 이동 거리(2026-11-03 결정, 테이프 추가 없음):
  가로선 노드(정차)  세 채널 모두 테이프(node)가 node_confirm 샘플 연속
  옆 선이 있는 노드  가운데 + 옆 선 쪽 채널이 테이프(분기·모서리·T자)
  odom 이동 거리가 예상 거리의 gate_min~gate_max(+slack) 안일 때만 노드로 인정한다.
  단 창 시작은 예상 위치 − gate_early_cm(5 cm)보다 앞서지 않는다(짧은 구간 이른 오감지 방지).
  예상 거리를 gate_max 넘게 지나도 노드가 없으면 정지(node_missed).
노드에서: 직진은 방향을 유지하며 pass_cm 통과, 회전은 turn_advance_cm(IR ↔ 바퀴 축 + 테이프 반폭)
  더 간 뒤 제자리 회전 → 가운데 채널이 새 선 위에 오고 좌우가 고르면(|오차| <= align_error) 다음 구간.
도착: load = 반시계 90° → 상차 완료(load_done)까지 대기 → 반시계 회전해 진입선을 찾음,
  park = 반시계 회전으로 진입선 확인(back_min_deg 이상) → 후진해 세 채널이 가로선을 보면 정지,
  handoff = 정지(Nav2 전환 가능), stop = 정지.
이어서 출발: 직전 경로가 끝난 노드에서 다시 출발하면 끝난 방향을 기억해 둔다. 첫 구간이 반대쪽이면
  (N12 handoff 뒤 되돌아가기 등) 먼저 반시계로 돌아 진입선을 찾고, 센서 위치(출발 노드 → 센서 거리)도
  끝난 동작으로 정한다. 정지·오류 뒤에는 기억을 지운다(사람이 옮겼을 수 있음).
  기억이 없고 출발 노드가 가로선 노드이면 센서가 가로선 위에 있다고 보고(0 cm) 거리 창을 쓴다.
경유 정차: 경로 중간의 가로선 노드(예: P5 → P4 상차 → P7)에서는 그 노드의 도착 동작(상차 대기 등)을 한 뒤
  같은 규칙(이어서 출발)으로 다음 구간을 이어 간다. 경로는 가로선 노드마다 구간 묶음으로 나눠 검사한다.
속도: set_speed 로 직선·회전 속도를 바꾼다(주행 중에도 다음 샘플부터). 상한은 설정의 max_linear·max_angular.
회전 직후 감속: 노드에서 회전한 뒤 이어지는 구간은 처음 post_turn_slow_cm(기본 10 cm)까지 approach_speed 이하.
노드 앞 감속: 직선 추종 중 이동 거리가 예상 노드 위치 - approach_lead_cm(기본 5 cm)를 넘으면, 그리고 노드
  감지 뒤 회전 전 전진·후진은 approach_speed(base_speed 보다 크면 base_speed) 이하로 간다. 출발 위치를 몰라
  예상 위치가 없는 구간은 처음부터 approach_speed. 직진 통과 노드(pass_full_speed)는 감속하지 않는다. 실로봇 d660 0.06 m/s(2026-11-06)에서 노드를
  지나쳐 멈춰 회전이 107~117° 로 끝나고 비스듬히 출발한 문제 대응(0.03 m/s 는 70~93°).
정지 명령·오류(라인 놓침 지속, 노드 놓침, 회전 중 선 못 찾음, 후진 초과, odom·IR 없음)는 모두 0 속도.
정밀 도킹(FFT1-112): 상차 노드별 값 (a, r, b) = (advance_cm, rotate_deg, move_cm), 기본 (0, 90, 0) = 위 load 동작.
  가로선 정지 → a ≠ 0 이면 a 이동(접근 방향, 음수 = 후진) → 반시계 r° → b ≠ 0 이면 b 이동(새 방향) → 상차 대기 →
  b ≠ 0 이면 −b 이동 → a = 0 이면 반시계 회전해 진입선 찾기((180 − r)° 기준, 창 = turn_min·turn_max 와 같은 폭),
  a ≠ 0 이면 시계 r° → −a 이동(기본 정지 자세로) → 반시계 90 + turn_min ~ 90 + turn_max 로 진입선 찾기 → 끝·경유.
  도킹 이동은 odom 이 멈춘 뒤(구동 지연으로 남은 움직임을 먼저 흘려보냄) 시작 자세에서 부호 있는 거리로 재고,
  approach_speed 이하·끝으로 갈수록 감속(min_speed 까지)·방향 유지. 거리 초과·시간 초과·멈추지 않음은 오류 정지.
라인 유실 복구(FFT1-163): 추종 중 IR 3개가 모두 선을 놓치고 아직 다음 노드 감지 창(min_ok) 전이면
  GAP = 놓치기 전 gap_heading_window_cm 동안의 odom 방향(평균)을 유지해 approach_speed 이하로 직진(최대 gap_max_cm),
  IR 하나라도 gap_confirm 샘플 연속 선 위면 추종으로 돌아감(PD 이전 오차 지움, 옆 IR 이면 추종 PD 가 조향 —
  GAP 기능 전에도 IR 하나라도 선 위면 추종했으므로 판정 완화가 아님). 상태 이름은 follow 그대로(recovery 필드).
  GAP 시간 상한 = 2 + 2·gap_max_cm / min_speed(s), odom 이동 0.1 cm 미만이 lost_timeout 동안 이어지면 정지(진행 없음).
  gap_max_cm 안에 못 찾으면 기본(search_deg 0)은 바로 정지 'line_lost: gap N cm exceeded'. search_deg > 0 이면
  SEARCH(구간마다 한 번) = 멈춤 확인 → 놓친 지점까지 후진 → 제자리 좌 search_deg → 우 2·search_deg → 원래 방향으로
  돌며 가운데 IR 정렬(align_error)을 찾음 → 찾으면 그 자리에서 추종, 못 찾으면 'line_lost: gap/search failed'
  (2D 시뮬에서 재획득 0건 — 놓친 지점 회전이라 가운데 IR 옆 이동이 약 ±1 cm 뿐, 그래서 기본 끔). 감지 창 안(놓친 곳이 이미 창 안이거나 GAP 이 창에 들어감)에서는
  같은 방향·속도로 직진하며 그 노드의 기존 판정(가로선 = IR 3개, 옆선·T자 규칙, node_confirm)으로만 노드를 찾고,
  max_ok·gap_max_cm 를 넘으면 정지('line_lost: gap into <노드> window, node not found'). 창 안에서는 SEARCH 안 함.
  창 전 GAP·SEARCH 중에는 노드 판정 안 함. 구간 시작 suppress_cm 안, 통과·전진·회전·후진·도킹 이동 중 유실은 예전 그대로.
교차선 출발(start_cross): IR 3개가 route[0](가로선 노드 아님)을 지나는 선 위, 로봇은 start_facing 을 봄
  → ir_ahead_cm 전진(축을 노드 중심으로) → 첫 구간 방향이 다르면 선 찾기 회전 → 첫 구간(s0 = ir_ahead, gate_early 적용).
일시정지·이어 가기·라인 후진(FFT1-172, 2D 시뮬만 — 실로봇 미검증):
  pause = 0 속도·상태 PAUSED(지금 단계·구간·이동 거리·선 방향 기억 유지, 시간 제한 없음). odom 이 멈추면
  (|v| < 0.003 m/s·|ω| < 0.03 rad/s 가 0.3 s) paused_settled. resume = 멈춘 단계를 그대로 이어 감(타이머는 멈춘 시간만큼 밀림).
  back(to) = 지나온 노드 to 까지 왔던 길을 되감아 후진(정지 확인 뒤, 회전·GAP·SEARCH·도킹 이동·주차 후진 중 거부).
  후진 추종 = 상태 피드백. 전진 PD 를 거꾸로 쓰면 IR 이 축 앞(ir_ahead_cm)이라 우반면 영점(+|v|/d)이 생겨 불안정.
    θ̂ = odom yaw − ψ(선 방향, 지나온 구간의 선 위 샘플로 구함), ŷ = e_cm − d·sin θ̂ (e_cm = 오차 × ir_spacing_cm,
    + = 선이 센서 오른쪽 = 축이 선 왼쪽), ω = +k_y·ŷ − k_θ·θ̂, k_y = ωₙ²/|v|(cm/s), k_θ = 2ζωₙ.
    유도: 선 좌표에서 ẏ = v·sin θ, θ̇ = ω, v < 0 → θ̈ + k_θ·θ̇ + k_y·|v|·θ = 0 (k_y, k_θ > 0 이면 안정).
    구간 시작 suppress_cm 과 지나치는 직진 노드 옆선 근처에서는 ŷ 를 쓰지 않음(옆선이 오차를 치우침, 방향만 유지).
  목적 노드 판정 = 전진과 같은 판정(가로선·옆선·T자·node_confirm·옆선 켜짐 확인), 창 = 예상 거리 기준 같은 규칙. 판정 즉시 정지.
  끝 자세 = IR 이 노드 위(전진 도착 감지 자세), 이어서 출발할 때 s0 = half_line_cm.
  회전했던 노드는 판정 → 축을 노드 중심으로(ir_ahead − half_line 전진) → 그 회전을 반대로(직진 선이 있으면 선 찾기)
  → 기억한 선 방향(odom)으로 제자리 맞춤(rv_align, 모서리는 이것만으로 되돌림) → 다음 구간. 직진 노드는 멈추지 않고 지나감.
  출발 노드에서 reverse_short_cm 안이면 판정 대신 odom 거리만큼(센서를 노드 중심으로).
  상차 대기(LOADING)에서는 도킹 이동을 되돌린 뒤(−b → 시계 r° → −a) 같은 방식. 실패 = reverse_lost·reverse_node_missed·
  reverse_heading·reverse_timeout(0 속도 한 번, ERROR). 뒤쪽 장애물 감지 없음 — 관제가 후진 길·목적 노드가 빈 것을 확인한 뒤에만.
"""

import math
from collections import deque
from dataclasses import dataclass, replace

from roport_line_nav.dock import DockTable
from roport_line_nav.graph import (HANDOFF, LEFT, LOAD, PARK, RIGHT, STOP, STRAIGHT, GraphError,
                                   BACK, Leg, STRAIGHT as GO_STRAIGHT, plan_route, turn_of)
from roport_pinky_line.follower import HARD_MAX_ANGULAR, HARD_MAX_LINEAR

IDLE = 'idle'
FOLLOW = 'follow'
PASS = 'pass'             # 직진 노드 통과(방향 유지)
ADVANCE = 'advance'       # 회전 전 바퀴 축을 노드 위로
ROTATE = 'rotate'
LOADING = 'loading'       # 상차 완료 대기
REVERSE = 'reverse'       # 대기·충전 칸으로 후진
ARRIVED = 'arrived'
HANDOFF_STATE = 'handoff'
STOPPED = 'stopped'
ERROR = 'error'
TERMINAL = (IDLE, ARRIVED, HANDOFF_STATE, STOPPED, ERROR)
FACINGS = ('E', 'N', 'W', 'S')

# 도킹 이동(부호 있는 ADVANCE): odom 정지 판정·종료 조건
SETTLE_CM = 0.02          # 샘플 사이 이동이 이보다 작고
SETTLE_DEG = 0.2          # 회전이 이보다 작은 새 odom 샘플 비교가 node_confirm 번 이어지면 멈춘 것으로 봄
SETTLE_TIMEOUT_S = 1.5    # 0 명령 뒤 이 시간 안에 멈추지 않으면 오류
MOVE_GAIN = 0.01          # m/s per 남은 cm (3 cm 남으면 0.03)
MOVE_OVERRUN_CM = 2.0     # 목표보다 이만큼 더 가면(부호 있는 거리) 오류
GAP_CHORD_MIN_CM = 2.0    # 선 위치 점으로 방향을 잴 최소 기억 거리
GAP_PROGRESS_CM = 0.1     # GAP 중 이만큼도 못 가고 lost_timeout 이 지나면 정지(진행 없음)
PAUSED = 'paused'         # 일시정지(FFT1-172): 단계·구간 기억 유지, 0 속도
REVERSE_FOLLOW_MIN = 0.01  # m/s, 라인 후진 속도 범위(2026-11-12 사용자 결정 0.01~0.03, 기본 0.02)
REVERSE_FOLLOW_MAX = 0.03
STILL_V = 0.003           # m/s, 일시정지 정지 확인: 선속 |v| 이 미만
STILL_W = 0.03            # rad/s, 각속 미만
STILL_S = 0.3             # s, 이 시간 이어지면 정지로 봄
# 정지 확인은 메시지 한 쌍의 속도가 아니라 STILL_S 창 안 자세 변화로 본다(2026-11-12 실로봇: 서 있어도 엔코더가
# 가끔 1틱 튀어 메시지 간 각속이 0.03 rad/s 를 넘음 → 정지 확인이 계속 풀려 back 후보가 비었음).
# 창 안 허용 변화 = 위 속도 × STILL_S 와 같은 크기(0.003 m/s × 0.3 s ≈ 0.1 cm, 0.03 rad/s × 0.3 s ≈ 0.52°).
STILL_POS_CM = 0.1
STILL_YAW_RAD = STILL_W * STILL_S
# 후진용 선 방향: 구간의 깨끗한(노드 주변 아닌) 선 위 샘플 최근 이 거리. 12 cm 는 IR 불감대(±0.4 cm) 안에서
# 천천히 흐르는 추종을 선 방향으로 봐 2D 시뮬에서 1.6° 틀림
PSI_WINDOW_CM = 30.0
BACK_MOVED_CM = 2.0       # 끝난 뒤 back: 감지(도착) 자세에서 odom 이 이보다 움직였으면 거절(옮겨졌을 수 있음)
BACK_TURNED_DEG = 5.0     # 끝난 뒤 back: 방향이 이보다 바뀌었으면 거절
PRE_CENTER_MARGIN_CM = 1.0       # 후진 끝 회전 출발 전 축 맞춤 이동 상한 = ir_ahead − 반폭 + 이 값
REVERSE_ACQUIRE_MARGIN_CM = 1.5  # 직진 선 없는 노드 되돌린 뒤 선 다시 잡기 여유(센서 → 테이프 거리 + 이 값)
RV_ALIGN_DEG = 2.0        # 후진 시작 전 odom 방향이 선 방향과 이보다 다르면 제자리로 맞춤
REVERSE_BLIND_CM = 2.0    # 지나치는 직진 노드 옆선 ± (half_line + 이 값) 에서는 ŷ 를 쓰지 않음


@dataclass(frozen=True)
class NavConfig:
    base_speed: float = 0.03       # m/s, 직선 추종
    min_speed: float = 0.015       # m/s
    max_linear: float = 0.06       # m/s
    max_angular: float = 0.6       # rad/s
    kp: float = 1.0                # rad/s per 오차 1
    kd: float = 0.05
    # 속도 맞춤 kp(2026-11-07 시험판): 직선 속도가 gain_ref_speed 를 넘으면 kp × 속도/gain_ref_speed.
    # kp 는 rad/s 라 같은 오차에서 1 cm 당 방향 수정이 속도에 반비례(0.08 은 0.06 의 3/4)
    # → 0.08 m/s d660 에서 회전 뒤 10° 비스듬히 30 cm 끌려가다 반대쪽으로 넘어가 이른 옆선 오감지.
    # 0.06 까지는 실로봇 완주(0.03·0.06)한 kp 그대로. kd 항은 오차 변화율(∝ 속도)이라 그대로 둠.
    kp_speed_scale: bool = True
    gain_ref_speed: float = 0.06   # m/s
    slow_gain: float = 0.5
    heading_kp: float = 2.0        # 통과·전진·후진 중 방향 유지 (rad/s per rad)
    ir_ahead_cm: float = 3.0       # 가운데 IR ↔ 바퀴 축 (2026-11-03 실측 [26])
    half_line_cm: float = 1.2      # 테이프 24 mm 반폭
    turn_advance_cm: float = 4.2   # 옆 선 감지 → 바퀴 축이 옆 선 중심에 올 때까지 (ir_ahead + half_line)
    pass_cm: float = 5.0           # 직진 노드 통과 거리
    suppress_cm: float = 4.0       # 구간 시작 직후 노드 감지 안 함
    gate_min: float = 0.7          # 예상 거리 비율 (±30%, 2026-11-03 결정)
    gate_max: float = 1.3
    gate_slack_cm: float = 3.0
    # 감지 창 시작은 「예상 위치 − gate_early_cm」보다 앞서지 않게(2026-11-07 사용자 결정 1안).
    # 실로그 61건의 진짜 노드는 예상 위치 −3.4~+2.9 cm. N14→N02 모서리(진짜 18.3~18.5 cm, 0.03 5회)를
    # 0.06·0.08 에서 왼쪽 센서가 본선 가장자리를 밟아 10.9~11.2 cm 에서 오감지(3회) → 회전 실패.
    # 비율 창(gate_min)만으로는 짧은 구간 시작이 약 11 cm 라 이를 거르지 못함. 0 이면 끔.
    # 경로 안에서 이어지는 구간(회전·통과·상차·주차 뒤)에만 적용. 사람이 놓은 첫 구간은 출발 위치가
    # 가정값이라(시뮬: N01·N11 칸 7 cm 안쪽 출발에서 진짜 노드를 놓침) 비율 창만 쓴다.
    gate_early_cm: float = 5.0
    rotate_speed: float = 0.4      # rad/s
    rotate_min_speed: float = 0.15
    turn_min_deg: float = 60.0     # 90° 회전에서 새 선을 찾기 시작하는 각도
    turn_max_deg: float = 120.0    # 이 각도까지 못 찾으면 정지
    back_min_deg: float = 172.0    # 180° 회전(대기 칸·되돌아 출발)에서 선을 찾기 시작하는 각도
    back_max_deg: float = 210.0
    align_error: float = 0.25      # 회전 중 가운데 채널이 선 위이고 |오차|가 이 값 이하이면 정렬로 봄
    lost_timeout: float = 1.0      # s, 라인 놓침이 이보다 길면 정지(오류)
    node_confirm: int = 2          # 연속 샘플
    # 줄기 쪽에서 T자(양쪽 갈래)로 들어갈 때는 왼쪽·오른쪽 센서가 둘 다 tee_window_cm 안에서 선을 봐야 노드.
    # 2026-11-07 d660: P5→N14 에서 왼쪽 센서만 0.3 cm(4 샘플) 테이프 가장자리 → N14 오감지 → 회전 실패.
    # 실로그 16건의 진짜 T자 감지는 모두 양쪽이 켜짐(각 1.0~2.8 cm).
    tee_both: bool = True
    tee_window_cm: float = 1.5
    # 옆선 켜짐 확인(2026-11-09 사용자 결정): 옆선 노드는 그 옆 IR 이 감지 창 시작(min_ok) 뒤에 꺼짐 → 켜짐으로
    # 바뀐 run 일 때만 인정. 창 전부터 켜져 있던 run(본선 가장자리 타기)은 무시. 꺼짐은 side_rise_off_samples
    # 연속이어야 run 이 끝난 것으로 봄(깜빡임). d660 0.08 m/s 11-09 21:54 N02→N14: 회전 직후 오른쪽으로 밀려
    # 왼쪽 IR 이 8.0 cm 부터 본선 가장자리를 계속 밟다가 창(14.8 cm)이 열리자 15.1 cm 오감지(진짜 약 18 cm).
    side_rise_only: bool = True
    side_rise_off_samples: int = 2
    side_rise_tee: bool = False    # T자(tee_both)에도 같은 규칙 — 실로그 재생으로 판단(README)
    reverse_speed: float = 0.02    # m/s
    reverse_max_cm: float = 12.0
    # 노드 앞 감속(2026-11-06 d660: 0.06 m/s 에서 노드를 지나쳐 멈춤, 0.03 m/s 는 완주).
    # 실제 값 = min(approach_speed, base_speed). base_speed 를 바꿔도 이 값은 그대로.
    approach_speed: float = 0.03   # m/s, 0.03 = 실로봇 완주 속도(2026-11-04·11-06)
    # 예상 노드 위치 몇 cm 전부터 감속(2026-11-07 사용자: 노드 3~5 cm 전). 실로그 61건의 진짜 감지는 예상 위치
    # -3.4 ~ +2.9 cm → 5 cm 면 모두 감속 뒤 감지, 3 cm 면 일찍 오는 노드(-3.1·-3.4)를 빠른 속도로 만남
    approach_lead_cm: float = 5.0
    # 노드에서 회전한 직후 구간 처음 이 거리(cm)는 approach_speed 이하. 0 = 끔.
    # 근거(2026-11-10 결정): 회전 뒤 선 옆으로 0.5~1 cm 밀린 채 0.06~0.08로 가면 옆 IR이 본선 가장자리를
    # 11.5~15.9 cm까지 밟아 감지 창과 겹침(오감지). 0.03 기록은 가장자리 타기가 8.5 cm 전에 끝남.
    post_turn_slow_cm: float = 10.0
    # 직진 통과 노드(N15·N23 등)는 다가갈 때·지날 때 감속하지 않음(2026-11-07 사용자)
    pass_full_speed: bool = True
    # 라인 유실 복구(FFT1-163, 시뮬만 확인 — 실로봇 미검증). gap_max_cm 0 = 끔(예전 동작)
    gap_max_cm: float = 8.0             # 놓친 뒤 방향 유지 직진 최대 거리
    gap_heading_window_cm: float = 4.0  # 놓치기 전 이 거리 동안의 odom 방향 평균을 유지
    gap_confirm: int = 2                # IR 하나라도 이 샘플 수 연속 선 위면 추종 복귀(창 안은 노드 판정 먼저)
    # SEARCH 제자리 회전 폭(좌 → 우 2배 → 원래 방향). 0 = 끔(기본): GAP 이 넘치면 후진·회전 없이 정지.
    # 2D 시뮬에서 SEARCH 재획득 0건(2026-11-09 검토 M1)이라 기본 끔
    search_deg: float = 0.0
    # 가운데 ↔ 옆 IR 간격(GAP 방향용 선 위치 추정). Pinky Pro HW Spec 약 20 mm, 실측 아님
    ir_spacing_cm: float = 2.0
    # 라인 후진(FFT1-172). 주차 후진 reverse_speed(P5·P6 0.02)와 따로. 실로봇 미검증
    reverse_follow_speed: float = 0.02   # m/s, REVERSE_FOLLOW_MIN ~ min(REVERSE_FOLLOW_MAX, max_linear)
    reverse_wn: float = 0.8              # rad/s, 후진 상태 피드백 고유진동수 ωₙ
    reverse_zeta: float = 0.8            # 감쇠비 ζ
    reverse_lost_cm: float = 2.0         # IR 3개 모두 놓침이 이 거리 이어지면 reverse_lost
    reverse_max_heading_deg: float = 15.0  # |θ̂| 가 넘으면 reverse_heading
    reverse_short_cm: float = 6.0        # IR 이 출발 노드에서 이 거리 미만이면 노드 판정 대신 odom 거리 후진

    def __post_init__(self):
        if not 0.0 < self.max_linear <= HARD_MAX_LINEAR:
            raise ValueError(f'max_linear must be in (0, {HARD_MAX_LINEAR}]: {self.max_linear}')
        if not 0.0 < self.max_angular <= HARD_MAX_ANGULAR:
            raise ValueError(f'max_angular must be in (0, {HARD_MAX_ANGULAR}]: {self.max_angular}')
        if not 0.0 < self.min_speed <= self.base_speed <= self.max_linear:
            raise ValueError('need 0 < min_speed <= base_speed <= max_linear')
        if not 0.0 < self.reverse_speed <= self.max_linear:
            raise ValueError('need 0 < reverse_speed <= max_linear')
        # 실제 감속 속도는 min(approach_speed, base_speed) ≤ max_linear 라 상한은 하드 상한으로만 본다.
        # trial launch 가 max_linear 를 직선 속도(최소 0.02)로 낮춰도 기본 0.03 으로 시작되게 한다.
        if not (math.isfinite(self.approach_speed)
                and self.min_speed <= self.approach_speed <= HARD_MAX_LINEAR):
            raise ValueError(f'need min_speed <= approach_speed <= {HARD_MAX_LINEAR}')
        if not (math.isfinite(self.approach_lead_cm) and 0.0 <= self.approach_lead_cm <= 50.0):
            raise ValueError('need 0 <= approach_lead_cm <= 50')
        if not (math.isfinite(self.post_turn_slow_cm) and 0.0 <= self.post_turn_slow_cm <= 50.0):
            raise ValueError('need 0 <= post_turn_slow_cm <= 50')
        if not 0.0 < self.rotate_min_speed <= self.rotate_speed <= self.max_angular:
            raise ValueError('need 0 < rotate_min_speed <= rotate_speed <= max_angular')
        if not (math.isfinite(self.gate_early_cm) and 0.0 <= self.gate_early_cm <= 50.0):
            raise ValueError('need 0 <= gate_early_cm <= 50')
        if not 0.0 < self.gate_min < 1.0 < self.gate_max:
            raise ValueError('need 0 < gate_min < 1 < gate_max')
        if not 0.0 < self.turn_min_deg < 90.0 < self.turn_max_deg <= 180.0:
            raise ValueError('need 0 < turn_min_deg < 90 < turn_max_deg <= 180')
        if not 150.0 <= self.back_min_deg < 180.0 < self.back_max_deg <= 240.0:
            raise ValueError('need 150 <= back_min_deg < 180 < back_max_deg <= 240')
        if not 0.0 < self.align_error < 1.0:
            raise ValueError('need 0 < align_error < 1')
        if not (math.isfinite(self.gain_ref_speed) and 0.0 < self.gain_ref_speed <= HARD_MAX_LINEAR):
            raise ValueError(f'need 0 < gain_ref_speed <= {HARD_MAX_LINEAR}')
        if not (math.isfinite(self.kp) and math.isfinite(self.kd) and self.kp > 0 and self.kd >= 0):
            raise ValueError('need kp > 0, kd >= 0')
        if not (math.isfinite(self.tee_window_cm) and 0.0 < self.tee_window_cm <= 5.0):
            raise ValueError('need 0 < tee_window_cm <= 5')
        if not (math.isfinite(self.gap_max_cm) and 0.0 <= self.gap_max_cm <= 15.0):
            raise ValueError('need 0 <= gap_max_cm <= 15')
        if not (math.isfinite(self.gap_heading_window_cm)
                and 0.5 <= self.gap_heading_window_cm <= 20.0):
            raise ValueError('need 0.5 <= gap_heading_window_cm <= 20')
        if not (math.isfinite(self.search_deg) and 0.0 <= self.search_deg <= 45.0):
            raise ValueError('need 0 <= search_deg <= 45')
        if not 1 <= self.side_rise_off_samples <= 10:
            raise ValueError('need 1 <= side_rise_off_samples <= 10')
        if not (math.isfinite(self.ir_spacing_cm) and 0.5 <= self.ir_spacing_cm <= 5.0):
            raise ValueError('need 0.5 <= ir_spacing_cm <= 5')
        if not (math.isfinite(self.reverse_follow_speed) and REVERSE_FOLLOW_MIN
                <= self.reverse_follow_speed <= min(REVERSE_FOLLOW_MAX, self.max_linear)):
            raise ValueError(f'need {REVERSE_FOLLOW_MIN} <= reverse_follow_speed <= '
                             f'min({REVERSE_FOLLOW_MAX}, max_linear)')
        if not (math.isfinite(self.reverse_wn) and 0.2 <= self.reverse_wn <= 2.0):
            raise ValueError('need 0.2 <= reverse_wn <= 2.0')
        if not (math.isfinite(self.reverse_zeta) and 0.3 <= self.reverse_zeta <= 1.5):
            raise ValueError('need 0.3 <= reverse_zeta <= 1.5')
        if not (math.isfinite(self.reverse_lost_cm) and 0.5 <= self.reverse_lost_cm <= 5.0):
            raise ValueError('need 0.5 <= reverse_lost_cm <= 5')
        if not (math.isfinite(self.reverse_max_heading_deg)
                and 5.0 <= self.reverse_max_heading_deg <= 30.0):
            raise ValueError('need 5 <= reverse_max_heading_deg <= 30')
        if not (math.isfinite(self.reverse_short_cm) and 0.0 <= self.reverse_short_cm <= 15.0):
            raise ValueError('need 0 <= reverse_short_cm <= 15')
        if self.gap_confirm < 1:
            raise ValueError('need gap_confirm >= 1')
        if self.node_confirm < 1 or self.reverse_max_cm <= 0 or self.lost_timeout <= 0:
            raise ValueError('node_confirm >= 1, reverse_max_cm > 0, lost_timeout > 0')


@dataclass(frozen=True)
class Command:
    linear: float
    angular: float


def _clamp(v, limit):
    return max(-limit, min(limit, v))


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class RouteExecutor:
    def __init__(self, graph, config=None, dock=None):
        self.graph = graph
        self.cfg = config or NavConfig()
        self.dock = dock if dock is not None else DockTable(graph)
        self._dock_phase = None  # 상차 도착 동작 중 어디인지(표시용): a·r·b·load·-b·unr·-a·back·back_a
        self._dock_disp = (0.0, 0.0)   # 표시용 도킹 이동 (접근 방향 cm, 새 방향 cm)
        self._gap = None          # GAP 중: {'d', 'yaw', 'pose', 'hits'}
        self._recovery = None     # SEARCH 중: {'d', 'window', 'yaw'}
        self._searched = False    # 이 구간에서 SEARCH 를 이미 했는지(구간마다 한 번)
        self._yaw_hist = deque()  # 추종 중 (이동 거리, odom yaw) — GAP 방향
        self.state = IDLE
        self.reason = ''
        self.route = []
        self.legs = []
        self.index = 0
        self.events = deque(maxlen=64)
        self.event_seq = 0       # 지금까지 기록한 이벤트 수(events 는 최근 64개만)
        self.route_seq0 = 0      # 현재 경로를 시작할 때의 event_seq
        self._steps = deque()
        self._zero_once = False
        self._odo = 0.0          # odom 누적 이동 거리 cm
        self._yaw_acc = 0.0      # odom 누적 회전 rad (감기지 않음)
        self._pose = None
        self._pose_stamp = None
        self._now = 0.0
        self._loaded = False     # 상차 완료를 받았는지
        self._leg_s0 = 0.0       # 구간 시작 때 출발 노드 → 센서 거리(표시용 위치 추정)
        self._progress = 0.0     # 현재 구간 진행률 0~1 (정지·오류 뒤에도 마지막 값 유지)
        self._seg_ends = set()   # 경유 정차 구간 묶음의 끝 Leg 번호
        self._end = None         # 직전 경로가 끝난 곳 {'node', 'facing', 's0', 'pose'} (이어서 출발용)
        self._base = 0           # 이어 붙인 직전 경로 구간 수(legs[:_base] = 지나온 길, legs[_base] = 이번 첫 구간)
        self._expected = None    # 지금 구간의 예상 노드 위치(센서 이동 거리 cm). 모르면 None
        self._slow_until = 0.0   # 이 구간에서 이 이동 거리(cm)까지 감속(회전 직후)
        self._pre = None         # 출발 전 제자리 회전의 시작 방향(도, 표시용)
        # 일시정지·라인 후진(FFT1-172)
        self._paused_from = None  # PAUSED 직전 상태
        self._pause_t0 = 0.0
        self._pause_noted = False  # paused_settled 이벤트를 남겼는지
        self._still_last = None   # 정지 확인: (시각, 자세)
        self._still_since = None  # 멈춘 상태가 시작된 시각
        self._still_hist = deque()  # 정지 확인 창: (시각, 자세), 최근 STILL_S 남짓
        self._back = None         # 후진 진행·결과 표시 {'to', 'phase', 'leg', ...}
        self._leg_ref = 0.0       # 구간 시작 때 _odo (구간 위치 = _leg_s0 + _odo − _leg_ref)
        self._leg_guess = True    # 구간 시작 위치가 가정값인지
        self._leg_begun = -1      # begin_leg 를 한 구간 번호
        self._leg_psi = {}        # 구간 번호 → 선 방향(odom yaw, rad)
        self._map_off = None      # odom yaw − 지도 방향(마지막으로 구한 구간 기준)
        self._arrive_info = None  # 지금 구간 끝 노드 감지 때 {'index', 'pos', 'pose'}
        self._psi_hist = deque()  # 이 구간의 깨끗한 선 위치 점 (d, x, y)
        self._rv_det = None       # 후진 중 마지막 노드 판정 때 odom 자세
        self._reset_primitive()

    # ------------------------------------------------------------ 명령
    def start(self, route, start_offset_cm=None, start_cross=False, start_facing=None):
        """경로 시작. 잘못된 경로는 GraphError(현재 상태 유지). 주행 중이면 거부.

        start_cross: IR 3개가 route[0] 을 지나는 교차선 위, 로봇은 start_facing(E·N·W·S)을 봄.
        """
        if self.state not in TERMINAL:
            raise GraphError(f'busy ({self.state}); stop first')
        legs, seg_ends = self._plan_segments([str(r) for r in route])
        if start_cross:
            self._check_cross(str(route[0]), start_facing, start_offset_cm)
        elif start_facing is not None:
            raise GraphError('start_facing is only for start_cross')
        else:
            self._check_pre_center(legs[0])
        route = [str(r) for r in route]
        # 끝난 뒤 odom 이 2 cm·5° 넘게 바뀌었거나 odom 을 모르면 끝 기억(s0·gate_early·pre_center·이어 붙이기)을 버리고
        # 손으로 놓은 출발과 같게(2026-11-12 결정 1). 회전 출발 거절(R1, 위 _check_pre_center)은 그대로
        dropped = None
        if self._end is not None and self._end_moved() is not False:
            dropped = self._end['node']
            self._end = None
        # 끝 기억 노드에서 이어 출발: 직전 경로의 지나온 길을 앞에 붙여 back 이 그 노드들까지 되감을 수 있게(FFT1-172 후속)
        chain = None if start_cross or start_offset_cm is not None else self._chain_prefix(legs[0])
        base, psi, map_off = 0, {}, None
        if chain is not None:
            p_route, p_legs, p_ends, psi = chain
            base = len(p_legs)
            legs = p_legs + legs
            seg_ends = p_ends | {base + e for e in seg_ends}
            map_off = self._map_off       # 끝난 뒤 옮겨지지 않았으므로(_chain_ok) odom − 지도 차이 그대로
            self.route = p_route + route[1:]
        else:
            self.route = route
        self.legs = legs
        self._seg_ends = seg_ends
        self.index = base
        self._base = base
        self.reason = ''
        self._loaded = False
        self._steps.clear()
        self._pre = None
        self._paused_from = None
        self._back = None
        self._leg_psi = psi
        self._map_off = map_off
        self._arrive_info = None
        self._leg_begun = -1
        self.route_seq0 = self.event_seq
        self._log('start', ' → '.join(route))
        if dropped is not None:
            self._log('end_dropped', f'{dropped}: moved or odom unknown since the end')
        if chain is not None:
            self._log('chain', ' → '.join(chain[0]))
        self._dock_phase = None
        end = self._end
        self._end = None
        if start_cross:
            self._depart_cross(legs[base], start_facing)
            return
        self._depart(legs[base], end, start_offset_cm)

    def _end_moved(self):
        """끝 기억 자세(_end['pose'])에서 odom 이 BACK_MOVED_CM 또는 BACK_TURNED_DEG 넘게 바뀌었는지. 모르면 None."""
        end = self._end
        ref = None if end is None else end.get('pose')
        pose = self._still_last[1] if self._still_last is not None else None   # odom 끊김 = 모름
        if ref is None or pose is None:
            return None
        moved = math.hypot(pose[0] - ref[0], pose[1] - ref[1])
        turned = abs(_wrap(pose[2] - ref[2]))
        return moved > BACK_MOVED_CM or turned > math.radians(BACK_TURNED_DEG)

    def _chain_ok(self):
        """직전 경로를 이어 붙일 수 있는 끝 기억인지(부작용 없음).

        ARRIVED 이고(끝난 뒤 stop 이면 _end 가 없음) 끝 노드가 가로선 정차 노드가 아니며(= 라인 후진으로 끝남, 경유 정차는
        되감기가 넘지 않음) 그 노드로 들어온 구간이 통과·회전 구간이고, 끝난 자세에서 2 cm·5° 안일 때만.
        """
        end = self._end
        if end is None or self.state != ARRIVED or not self.legs:
            return False
        if len(self.route) != len(self.legs) + 1 or self.route[-1] != end['node']:
            return False
        if self.graph.nodes[end['node']].bar or self.legs[-1].to != end['node']:
            return False
        if self.legs[-1].action not in (STRAIGHT, LEFT, RIGHT):
            return False
        return self._end_moved() is False

    def _chain_prefix(self, leg0):
        """이어 붙일 직전 경로. → (route, legs, 경유 정차 끝 번호, 선 방향 기억) 또는 None(부작용 없음).

        이음 노드에서의 회전 = (직전 경로가 그 노드로 들어온 방향 → 새 첫 구간 방향). U턴이면 붙이지 않음.
        """
        if not self._chain_ok() or leg0.frm != self._end['node']:
            return None
        last = self.legs[-1]
        turn = turn_of(last.heading, leg0.heading)
        if turn not in (GO_STRAIGHT, LEFT, RIGHT):
            return None
        n = len(self.legs)
        legs = self.legs[:-1] + [replace(last, action=turn)]
        ends = {k for k in self._seg_ends if k < n - 1}
        psi = {k: v for k, v in self._leg_psi.items() if k < n}
        return list(self.route), legs, ends, psi

    def _shown_base(self):
        """상태 보고의 route·index 기준. 이어 붙인 앞부분은 back 을 시작하기 전까지 보고하지 않음(명령한 경로 그대로)."""
        return 0 if self._back is not None else self._base

    @property
    def route_shown(self):
        """상태에 보고하는 경로(명령한 경로, back 뒤에는 이어 붙인 앞부분 포함)."""
        return self.route[self._shown_base():]

    def _end_dict(self):
        """상태 보고 end: {node, facing, moved, passed}. passed = 이 노드에서 이어 출발하면 붙는 지나온 노드
        (경로 순서, 마지막 경유 정차 노드까지 — back 이 넘지 않는 곳). 붙일 수 없으면 []. U턴 출발이면 붙이지 않음."""
        e = self._end
        if e is None:
            return None
        passed = []
        if self._chain_ok():
            n = len(self.legs)
            for k in range(n - 1, -1, -1):
                if k < n - 1 and k in self._seg_ends:
                    break
                passed.append(self.legs[k].frm)
            passed.reverse()
        return {'node': e['node'], 'facing': e['facing'], 'moved': self._end_moved(),
                'passed': passed}

    def _check_pre_center(self, leg):
        """후진 끝 기억에서 좌·우 회전 출발(pre_center)을 쓸 수 있는지. 아니면 기억을 버리고 GraphError(움직이지 않음).

        back 과 같은 기준: 후진 도착 자세에서 odom 이 BACK_MOVED_CM·BACK_TURNED_DEG 안. 이동 거리(노드 중심까지
        방향 투영)는 ir_ahead − 반폭 + PRE_CENTER_MARGIN_CM 이하.
        """
        end = self._end
        if end is None or end.get('center') is None or end['node'] != leg.frm \
                or turn_of(end['facing'], leg.heading) not in (LEFT, RIGHT):
            return
        c = self.cfg
        pose = self._still_last[1] if self._still_last is not None else self._pose
        ref = end.get('pose')
        why = None
        if pose is None or ref is None:
            why = 'odom unknown'
        else:
            moved = math.hypot(pose[0] - ref[0], pose[1] - ref[1])
            turned = abs(_wrap(pose[2] - ref[2]))
            cx, cy = end['center']
            cm = (cx - pose[0]) * math.cos(pose[2]) + (cy - pose[1]) * math.sin(pose[2])
            limit = c.ir_ahead_cm - c.half_line_cm + PRE_CENTER_MARGIN_CM
            if moved > BACK_MOVED_CM or turned > math.radians(BACK_TURNED_DEG):
                why = f'moved {moved:.1f} cm / {math.degrees(turned):.1f} deg since reverse arrival'
            elif abs(cm) > limit:
                why = f'center move {cm:+.1f} cm exceeds {limit:.1f} cm'
        if why is not None:
            self._end = None
            raise GraphError(f'start rejected: {why}; place the robot and stop, then start again')
        self._update_odom(pose)            # 끝난 동안 쌓지 않은 odom 을 첫 이동에 섞지 않게

    def _check_cross(self, nid, facing, offset):
        if facing not in FACINGS:
            raise GraphError(f'start_facing must be one of {"/".join(FACINGS)}')
        if offset is not None:
            raise GraphError('start_cross cannot be combined with start_offset_cm')
        node = self.graph.nodes[nid]
        if node.bar:
            raise GraphError(f'start_cross needs a line node, {nid} is a stop (bar) node')
        cross = [turn_of(facing, self.graph.heading(nid, o)) for o in node.links]
        if LEFT not in cross and RIGHT not in cross:
            raise GraphError(f'{nid} has no line across facing {facing}')

    def _depart_cross(self, leg, facing):
        """교차선 위 출발: 축을 노드 중심으로 → (필요하면) 선 찾기 회전 → 첫 구간(s0 = ir_ahead)."""
        c = self.cfg
        turn = turn_of(facing, leg.heading)
        self._log('start_cross', f'{leg.frm} facing {facing} → {leg.heading}')
        self._pre = self._HEADING_DEG[facing]
        self._steps.append((ADVANCE, c.ir_ahead_cm))
        if turn == BACK:
            self._steps.append((ROTATE, 1.0, c.back_min_deg, c.back_max_deg, True))
        elif turn in (LEFT, RIGHT):
            self._steps.append((ROTATE, 1.0 if turn == LEFT else -1.0,
                                c.turn_min_deg, c.turn_max_deg, True))
        self._steps.append(('begin', c.ir_ahead_cm))
        self._next_step()

    def set_speed(self, base_speed=None, rotate_speed=None, reverse_speed=None):
        """직선(m/s)·회전(rad/s)·라인 후진(m/s) 속도. 범위 밖이면 ValueError(그대로 유지).

        → 적용된 (직선, 회전). reverse_speed 는 라인 후진(reverse_follow_speed)이고 주차 후진(reverse_speed
        설정, P5·P6)은 바꾸지 않는다. 범위 = REVERSE_FOLLOW_MIN ~ min(REVERSE_FOLLOW_MAX, max_linear).
        """
        c = self.cfg
        base = c.base_speed if base_speed is None else float(base_speed)
        rot = c.rotate_speed if rotate_speed is None else float(rotate_speed)
        rv = c.reverse_follow_speed if reverse_speed is None else float(reverse_speed)
        if not (math.isfinite(base) and c.min_speed <= base <= c.max_linear):
            raise ValueError(f'base_speed must be in {c.min_speed}..{c.max_linear} m/s')
        if not (math.isfinite(rot) and c.rotate_min_speed <= rot <= c.max_angular):
            raise ValueError(
                f'rotate_speed must be in {c.rotate_min_speed}..{c.max_angular} rad/s')
        rv_max = min(REVERSE_FOLLOW_MAX, c.max_linear)
        if not (math.isfinite(rv) and REVERSE_FOLLOW_MIN <= rv <= rv_max):
            raise ValueError(f'reverse_speed must be in {REVERSE_FOLLOW_MIN}..{rv_max} m/s')
        self.cfg = replace(c, base_speed=base, rotate_speed=rot, reverse_follow_speed=rv)
        self._log('speed', f'{base:.3f} m/s, {rot:.2f} rad/s'
                  + ('' if reverse_speed is None else f', reverse {rv:.3f} m/s'))
        return base, rot

    def set_dock(self, node, advance_cm, rotate_deg, move_cm):
        """상차 노드 도킹 값 적용. 주행 중·범위 밖·상차 노드 아님이면 ValueError(그대로 유지)."""
        self._check_idle('dock')
        v = self.dock.set(node, advance_cm, rotate_deg, move_cm)
        self._log('dock', f'{node} a={v.advance_cm:g} r={v.rotate_deg:g} b={v.move_cm:g}')
        return v

    def reset_dock(self, node):
        self._check_idle('dock')
        self.dock.reset(node)
        self._log('dock', f'{node} reset')

    def replace_dock(self, table, detail=''):
        """검사·저장을 끝낸 표로 바꿈(노드가 파일 저장 뒤 적용할 때). 주행 중이면 ValueError."""
        self._check_idle('dock')
        self.dock = table
        self._log('dock', detail)

    def _check_idle(self, what):
        if self.state not in TERMINAL:
            raise ValueError(f'{what} rejected while driving ({self.state}); stop first')

    def _plan_segments(self, route):
        """중간 가로선 노드에서 나눠 구간 묶음마다 plan_route. → (전체 Leg 목록, 묶음 끝 Leg 번호)."""
        cuts = [i for i in range(1, len(route) - 1) if self.graph.nodes.get(route[i]) is not None
                and self.graph.nodes[route[i]].bar]
        legs, ends, a = [], set(), 0
        for b in cuts + [len(route) - 1]:
            legs += plan_route(self.graph, route[a:b + 1])
            ends.add(len(legs) - 1)
            a = b
        return legs, ends

    def _depart(self, leg, end, s0_override=None):
        """이 노드에서 leg 로 출발: 필요하면 출발 전 회전, 첫 구간 거리 창."""
        facing, pre, s0 = self._start_plan(leg, end)
        if s0_override is not None:
            s0 = s0_override
        if pre is None:
            # 기억한 끝 위치 없이 가정한 거리(사람이 놓은 출발·지정 오프셋)면 gate_early 를 걸지 않음
            guess = end is None or end['node'] != leg.frm or s0_override is not None
            self._begin_leg(s0, guess)
            return
        direction, lo, hi = pre
        self._log('pre_rotate', f'{facing} → {leg.heading}')
        self._pre = self._HEADING_DEG[facing]
        if end is not None and end.get('center') is not None \
                and turn_of(facing, leg.heading) in (LEFT, RIGHT):
            # 라인 후진으로 끝난 자세(축이 노드 중심 약 2 cm 뒤)에서 좌·우 회전 출발: 회전 전에 축을 노드 중심으로
            # (전진 도착의 turn_advance 와 같은 역할). 축이 중심이므로 첫 구간 센서 위치 = ir_ahead
            self._log('pre_center', f'{leg.frm}')
            self._steps.append((ADVANCE, 0.0, 'rv_pre', end['center']))
            s0 = self.cfg.ir_ahead_cm
        self._steps.extend([(ROTATE, direction, lo, hi, True), ('begin', s0)])
        self._next_step()

    def _start_plan(self, leg, end):
        """→ (기억한 방향, 출발 전 회전 (방향, 최소°, 최대°) 또는 None, 출발 노드 → 센서 거리 cm).

        센서 거리를 모르면 None(첫 구간 최소 거리 제한 없음).
        """
        c = self.cfg
        if end is None or end['node'] != leg.frm:
            return None, None, (0.0 if self.graph.nodes[leg.frm].bar else None)
        facing = end['facing']
        turn = turn_of(facing, leg.heading)
        if turn == GO_STRAIGHT:
            return facing, None, end['s0']
        if turn == BACK:       # 바퀴 축 기준 반 바퀴: 센서가 축 반대편으로
            return (facing, (1.0, c.back_min_deg, c.back_max_deg),
                    2 * c.ir_ahead_cm + c.half_line_cm)
        return facing, (1.0 if turn == LEFT else -1.0, c.turn_min_deg, c.turn_max_deg), None

    def stop(self, reason='stop command'):
        if self.state in TERMINAL and self.state != IDLE:
            self._end = None     # 끝난 뒤의 정지 = 로봇을 옮길 수 있으니 끝난 방향을 잊음
            return
        self._finish(STOPPED, reason)

    def load_done(self):
        """상차 완료. LOADING 에서만 받는다."""
        if self.state != LOADING:
            return False
        self._loaded = True
        self._log('load_done', self.legs[self.index].to)
        self._next_step()
        return True

    def fail(self, reason):
        if self.state in TERMINAL:
            return
        self._finish(ERROR, reason)

    # ------------------------------------------------------------ 일시정지·라인 후진(FFT1-172)
    def pause(self):
        """주행 중 일시정지: 0 속도, 지금 단계·구간·이동 거리 기억 유지. 끝난 뒤면 ValueError. → 결과 문자열."""
        if self.state == PAUSED:
            return 'already paused'
        if self.state in TERMINAL:
            raise ValueError(f'pause rejected: not driving ({self.state})')
        prev = self.state
        leg = self.legs[self.index]
        self._paused_from = prev
        self._pause_t0 = self._now
        self._pause_noted = False
        self.state = PAUSED
        self._log('pause', f'{prev} on {leg.frm}→{leg.to} at {self._odo - self._ref_odo:.1f} cm')
        return f'paused ({prev})'

    def resume(self):
        """PAUSED 에서만: 멈춘 단계를 그대로 이어 감(감지 창·누적 거리 그대로, 타이머는 멈춘 시간만큼 밀림)."""
        if self.state != PAUSED:
            raise ValueError(f'resume rejected: not paused ({self.state})')
        dt = max(0.0, self._now - self._pause_t0)
        self._shift_time(dt)
        self.state = self._paused_from
        self._paused_from = None
        self._prev_error = None
        self._prev_t = None
        self._hits = 0
        self._log('resume', f'{self.state} after {dt:.1f} s')
        return f'resumed ({self.state})'

    @property
    def settled(self):
        """odom 이 멈춤(|v| < STILL_V, |ω| < STILL_W 가 STILL_S 이어짐)."""
        return (self._still_since is not None and self._still_last is not None
                and self._still_last[0] - self._still_since >= STILL_S - 1e-9)

    def back(self, to):
        """지나온 노드 to 까지 라인 후진. 거절이면 ValueError(움직이지 않음). → 결과 문자열.

        받는 때: PAUSED(정지 확인 뒤, 직선 추종·통과·회전 전 전진·상차 대기에서 멈춤), 정지 확인 뒤의 LOADING,
        가로선 정지(STOP)로 끝난 ARRIVED(끝난 뒤 stop 으로 기억을 지우지 않았고, 감지 자세에서 2 cm·5° 안).
        HANDOFF(N12, Nav2 소유)·상차/주차 뒤 ARRIVED 는 거절.
        """
        to = str(to)
        src, pose = self._back_source()
        plan, j = self._back_plan(to, src, pose)
        if self.state in TERMINAL:
            self._update_odom(pose)       # 끝난 뒤에는 odom 을 안 쌓았으므로 최신 자세로
        i = self.index
        leg_i = self.legs[i]
        self._back = {'to': to, 'phase': 'undo_dock' if src == LOADING else 'start', 'from': src,
                      'leg': [self.legs[j].frm, leg_i.to], 'est': None}
        self._paused_from = None
        self._end = None
        self._steps.clear()
        self._steps.extend(plan)
        self._log('back', f'{src} {leg_i.frm}→{leg_i.to} → {to} '
                  f'({sum(1 for s in plan if s[0] == "rv_line")} reverse stretch)')
        self._next_step()
        return f'back to {to}'

    def back_targets(self):
        """지금 back 으로 받을 수 있는 노드(최근 지난 순). 받을 수 없으면 []."""
        try:
            src, pose = self._back_source()
        except ValueError:
            return []
        out = []
        for k in range(self.index, -1, -1):
            if k < self.index and k in self._seg_ends:
                break
            nid = self.legs[k].frm
            if nid in out:
                continue
            try:
                self._back_plan(nid, src, pose)
            except ValueError:
                continue
            out.append(nid)
        return out

    def back_why(self):
        """back 후보가 없을 때 그 이유(화면 표시용). 후보가 있거나 일시정지·도착·상차 대기가 아니면 None."""
        if self.state not in (PAUSED, LOADING, ARRIVED):
            return None
        try:
            src, pose = self._back_source()
        except ValueError as e:
            return str(e)
        if self.back_targets():
            return None
        for k in range(self.index, -1, -1):
            try:
                self._back_plan(self.legs[k].frm, src, pose)
            except ValueError as e:
                return str(e)
        return 'back rejected: no passed node'

    def _back_source(self):
        """back 을 받을 수 있는 상태인지 검사. → (시작 상태, 지금 odom 자세). 아니면 ValueError(부작용 없음)."""
        st = self.state
        if self._back is not None:
            raise ValueError('back rejected: back already done or in progress '
                             '(resume/stop, or start a new route)')
        if st == PAUSED:
            src = self._paused_from
        elif st in (LOADING, ARRIVED):
            src = st
        elif st == HANDOFF_STATE:
            raise ValueError('back rejected: after handoff the robot belongs to Nav2')
        else:
            raise ValueError(f'back rejected: needs paused, loading or arrived ({st})')
        if src == ROTATE:
            raise ValueError('back rejected: paused while rotating (resume or stop)')
        if self._gap is not None or self._recovery is not None:
            raise ValueError('back rejected: paused in line-gap recovery (resume or stop)')
        if src == ADVANCE and self._prim.get('signed'):
            raise ValueError('back rejected: paused in a dock move (resume or stop)')
        if src == REVERSE:
            raise ValueError('back rejected: paused in park reverse (resume or stop)')
        if self._pre is not None:
            raise ValueError('back rejected: paused before the first leg (start turn)')
        if not self.legs:
            raise ValueError('back rejected: no route')
        if not self.settled:
            raise ValueError('back rejected: robot still moving (wait for paused_settled)')
        pose = self._still_last[1] if st in TERMINAL else self._pose
        i = self.index
        if src == ARRIVED:
            leg_i = self.legs[i]
            if leg_i.action != STOP:
                raise ValueError(f'back rejected: after {leg_i.action} the robot faces away '
                                 'from the route; start a route instead')
            if self._end is None:
                raise ValueError('back rejected: position forgotten (stopped after arrival)')
            info = self._arrive_info
            if info is None or info['index'] != i:
                raise ValueError('back rejected: arrival position unknown')
            ax, ay, ayaw = info['pose']
            moved = math.hypot(pose[0] - ax, pose[1] - ay)
            turned = abs(_wrap(pose[2] - ayaw))
            if moved > BACK_MOVED_CM or turned > math.radians(BACK_TURNED_DEG):
                raise ValueError(f'back rejected: moved {moved:.1f} cm / '
                                 f'{math.degrees(turned):.1f} deg since arrival')
        elif src in (FOLLOW, PASS, ADVANCE) and self._leg_begun != i:
            raise ValueError('back rejected: leg not started')
        return src, pose

    def _back_plan(self, to, src, pose):
        """되감기 단계 목록(부작용 없음). → (단계, 목적 구간 번호 j). 못 하면 ValueError."""
        c = self.cfg
        i = self.index
        leg_i = self.legs[i]
        j = None
        for k in range(i, -1, -1):
            if k < i and k in self._seg_ends:     # 경유 정차(상차·주차)를 거꾸로 지나지 않음
                break
            if self.legs[k].frm == to:
                j = k
                break
        if j is None:
            raise ValueError(f'back rejected: {to} is not a node passed on this route '
                             '(since the last stop)')
        if src in (FOLLOW, PASS, ADVANCE):
            pos = self._leg_pos()
        elif src == ARRIVED:
            pos = self._arrival_pos(i, pose)
        else:
            pos = None                          # LOADING: 도킹 되돌림 뒤에 잰다
        plan = []
        if src == LOADING:
            plan += self._undo_dock_steps(leg_i)
        k, first, acquire = i, True, 0.0
        while True:
            k0, extra = k, 0.0
            while k > j and self.legs[k - 1].action == STRAIGHT:   # 직진 노드는 멈추지 않고 지나감
                extra += self.legs[k - 1].length
                k -= 1
            marks, cum = [], 0.0       # 지나치는 노드(직진 노드·출발 구간 끝)의 위치: 이 구간 목표 노드에서 cm
            for m in range(k, k0 + 1):
                cum += self.legs[m].length
                marks.append(cum)
            final = k == j
            pseudo = self._reverse_target(k)
            if pseudo is None:
                raise ValueError(f'back rejected: {self.legs[k].frm} has no line to detect '
                                 'when reversing')
            psi = self._psi_of(k0)
            short = pos is not None and pos + extra < c.reverse_short_cm
            if psi is None and not short:
                raise ValueError(f'back rejected: line direction of {self.legs[k0].frm}→'
                                 f'{self.legs[k0].to} unknown')
            if not short:
                plan.append(('rv_align', psi))
            plan.append(('rv_line', {
                'k0': k0, 'k': k, 'node': self.legs[k].frm, 'pos': pos, 'extra': extra,
                'marks': marks, 'guess': self._leg_guess if first else False, 'psi': psi,
                'final': final, 'pseudo': pseudo, 'arrival': i if pos is None else None,
                'acquire_cm': acquire}))
            if final:
                break
            t = self.legs[k - 1]                 # 이 노드에서 돌았던 회전을 반대로
            node = t.to
            direction = -1.0 if t.action == LEFT else 1.0
            ahead = any(self.graph.heading(node, o) == t.heading
                        for o in self.graph.nodes[node].links)
            # 직진 방향 선이 있으면 선 찾기 회전. 이어서 rv_align 이 지나온 구간의 선 방향(odom)으로 맞춤
            # (선 찾기는 70~93° 에서 끝날 수 있어 — 2D 시뮬 76° — 그대로 후진하면 14° 비스듬).
            # 직진 선이 없으면(모서리·분기에서 들어와 돈 T자) rv_align 이 선 방향까지 돌리고, 센서가 테이프 끝 너머
            # (ir_ahead − 반폭 ≈ 1.8 cm)에서 시작하므로 다음 구간 처음 acquire_cm 은 방향만 유지하며 선을 다시 잡음
            if ahead:
                plan.append((ROTATE, direction, c.turn_min_deg, c.turn_max_deg, True, 'rv_unturn'))
                acquire = 0.0
            else:
                acquire = c.ir_ahead_cm - c.half_line_cm + REVERSE_ACQUIRE_MARGIN_CM
            pos = t.length + c.ir_ahead_cm       # 축이 노드 중심 → 센서는 노드에서 ir_ahead
            k -= 1
            first = False
        plan.append(('rv_done', j))
        return plan, j

    @property
    def active(self):
        return self.state not in TERMINAL

    @property
    def approach_speed(self):
        """노드 앞·통과·전진에 쓰는 직선 속도 m/s(base_speed 를 넘지 않음)."""
        return min(self.cfg.approach_speed, self.cfg.base_speed)

    # ------------------------------------------------------------ 한 샘플
    def step(self, reading, pose, now, pose_stamp=None):
        """IR 한 샘플마다 호출. pose = (x_cm, y_cm, yaw_rad) 또는 None(odom 없음).

        pose_stamp: 그 pose 의 odom 수신 시각 등(같은 odom 이 다시 들어오면 같은 값). None 이면 샘플마다 새 pose 로 본다.
        도킹 이동 전 멈춤 확인은 stamp 가 바뀐 샘플끼리만 비교한다(odom 이 IR 보다 늦게 갱신돼도 멈춤으로 세지 않게).
        """
        self._now = now
        self._pose_stamp = pose_stamp
        self._watch_still(pose, now, pose_stamp)
        if self.state in TERMINAL:
            if self._zero_once:
                self._zero_once = False
                return Command(0.0, 0.0)
            return None
        if pose is None:
            self.fail('no_odom')
            return self.step(reading, pose, now)
        self._update_odom(pose)
        handler = {
            FOLLOW: self._follow, PASS: self._drive, ADVANCE: self._drive,
            ROTATE: self._rotate, LOADING: self._loading, REVERSE: self._reverse,
            PAUSED: self._paused,
        }[self.state]
        cmd = handler(reading)
        if self.state in TERMINAL:   # 이 샘플에서 끝남
            self._zero_once = False
            return Command(0.0, 0.0)
        return cmd

    # ------------------------------------------------------------ 상태 보고
    def as_dict(self):
        leg = self.legs[self.index] if self.legs and self.index < len(self.legs) else None
        out = {
            'state': self.state, 'reason': self.reason, 'route': self.route_shown,
            'index': self.index - self._shown_base(),
            'leg': None if leg is None else {
                'from': leg.frm, 'to': leg.to, 'heading': leg.heading, 'action': leg.action,
                'detect': leg.detect, 'sides': sorted(leg.sides)},
            'traveled_cm': round(self._odo - self._ref_odo, 1),
            'window_cm': None if self.state != FOLLOW else
            [round(self._min_ok, 1), round(self._max_ok, 1)],
            'rotated_deg': round(math.degrees(abs(self._yaw_acc - self._ref_yaw)), 1),
            'events': list(self.events),
            'event_seq': self.event_seq,
            'route_event_seq0': self.route_seq0,
            'dock': self.dock.as_dict(),
            'recovery': self._recovery_dict(),
            'pause': None if self.state != PAUSED else {
                'from': self._paused_from, 'settled': self.settled},
            'reverse': self._reverse_dict(),
            'back_targets': self.back_targets(),
            'back_why': self.back_why(),
            'end': self._end_dict(),
            'speed': {'base_speed': self.cfg.base_speed, 'rotate_speed': self.cfg.rotate_speed,
                      'min_speed': self.cfg.min_speed, 'max_linear': self.cfg.max_linear,
                      'rotate_min_speed': self.cfg.rotate_min_speed,
                      'max_angular': self.cfg.max_angular,
                      'approach_speed': self.approach_speed,
                      'approach_lead_cm': self.cfg.approach_lead_cm,
                      'post_turn_slow_cm': self.cfg.post_turn_slow_cm,
                      'pass_full_speed': self.cfg.pass_full_speed,
                      'kp': self.cfg.kp, 'kd': self.cfg.kd,
                      'kp_at_base': round(self.kp_for(self.cfg.base_speed), 3),
                      'kp_speed_scale': self.cfg.kp_speed_scale,
                      'gain_ref_speed': self.cfg.gain_ref_speed,
                      'reverse_follow_speed': self.cfg.reverse_follow_speed,
                      'reverse_follow_min': REVERSE_FOLLOW_MIN,
                      'reverse_follow_max': min(REVERSE_FOLLOW_MAX, self.cfg.max_linear)},
        }
        out.update(self._estimate(leg))
        return out

    _HEADING_DEG = {'E': 0.0, 'N': 90.0, 'W': 180.0, 'S': -90.0}

    def _recovery_dict(self):
        if self._gap is not None and self.state == FOLLOW:
            g = self._gap
            return {'mode': 'gap', 'lost_at_cm': round(g['d'], 1),
                    'gap_cm': round(self._odo - self._ref_odo - g['d'], 1),
                    'in_window': g['window']}
        if self._recovery is not None and self.active:
            return {'mode': 'search', 'lost_at_cm': round(self._recovery['d'], 1)}
        return None

    def _estimate(self, leg):
        """화면 표시용 바퀴 축 위치 추정(실측 cm). 그래프 + 이동 거리로 계산, 제어에는 쓰지 않는다."""
        est = self._back_estimate()
        if est is not None:
            return est
        if leg is None:
            return {'leg_progress': None, 'pose_est': None}
        if self._pre is not None and self.state in (ROTATE, ADVANCE):   # 출발 전 전진·제자리 회전
            n = self.graph.nodes[leg.frm]
            x, y = n.x, n.y
            if self.state == ROTATE:
                turned = math.degrees(abs(self._yaw_acc - self._ref_yaw))
                heading = self._pre + self._prim['dir'] * turned
            else:   # 교차선 출발: 축이 노드 중심 ir_ahead 뒤에서 앞으로
                heading = self._pre
                back = max(0.0, self._prim['cm'] - (self._odo - self._ref_odo))
                ux, uy = math.cos(math.radians(heading)), math.sin(math.radians(heading))
                x, y = n.x - ux * back, n.y - uy * back
            heading = (heading + 180.0) % 360.0 - 180.0
            self._progress = 0.0
            return {'leg_progress': 0.0, 'pose_est': {'x_cm': round(x, 1), 'y_cm': round(y, 1),
                                                      'heading_deg': round(heading, 1)}}
        if self.state == FOLLOW:
            axle = self._leg_s0 + (self._odo - self._ref_odo) - self.cfg.ir_ahead_cm
            self._progress = min(1.0, max(0.0, axle / leg.length)) if leg.length > 0 else 1.0
        elif self._recovery is not None and self.state in (ADVANCE, ROTATE):   # SEARCH: 놓친 지점
            axle = self._leg_s0 + self._recovery['d'] - self.cfg.ir_ahead_cm
            self._progress = min(1.0, max(0.0, axle / leg.length)) if leg.length > 0 else 1.0
        elif self.state in (PASS, ADVANCE, ROTATE, LOADING, REVERSE, ARRIVED, HANDOFF_STATE):
            self._progress = 1.0
        a, b = self.graph.nodes[leg.frm], self.graph.nodes[leg.to]
        heading = self._HEADING_DEG[leg.heading]
        f = self._progress
        x, y = a.x + (b.x - a.x) * f, a.y + (b.y - a.y) * f
        if leg.action == LOAD and self._dock_phase is not None and \
                self.state in (ADVANCE, ROTATE, LOADING, ARRIVED):
            x, y, heading = self._dock_estimate(leg, x, y, heading)
        elif leg.action == PARK and self.state in (REVERSE, ARRIVED):
            heading += 180.0
        elif self.state == ROTATE and self._prim:
            heading += self._prim['dir'] * math.degrees(abs(self._yaw_acc - self._ref_yaw))
        heading = (heading + 180.0) % 360.0 - 180.0
        return {'leg_progress': round(f, 3), 'pose_est': {
            'x_cm': round(x, 1), 'y_cm': round(y, 1), 'heading_deg': round(heading, 1)}}

    def _dock_estimate(self, leg, x, y, approach):
        """상차 도착 동작 중 표시 자세(바퀴 축): 가로선 정지 위치(노드 중심에서 접근 반대로 half_line + ir_ahead)
        + a·접근 방향 + b·(접근 + r) 방향, 방향 = 접근 + 회전. 정지 밀림(구동 지연)은 모른다."""
        r = self.dock.get(leg.to).rotate_deg
        da, db = self._dock_disp
        da -= self.cfg.half_line_cm + self.cfg.ir_ahead_cm
        phase = self._dock_phase
        p = self._prim
        moving = self.state == ADVANCE and p.get('signed')
        turned = math.degrees(abs(self._yaw_acc - self._ref_yaw)) if self.state == ROTATE else 0.0
        if moving and phase in ('a', '-a'):
            da += p['proj']
        elif moving:
            db += p['proj']
        if phase in ('a', '-a'):
            heading = approach
        elif phase == 'r':
            heading = approach + (turned if self.state == ROTATE else r)
        elif phase == 'unr':                  # 시계로 r° 되돌림
            heading = approach + (r - turned if self.state == ROTATE else 0.0)
        elif phase == 'back':
            heading = approach + (r + turned if self.state == ROTATE else 180.0)
        elif phase == 'back_a':
            heading = approach + (turned if self.state == ROTATE else 180.0)
        else:      # b·load·-b
            heading = approach + r
        u = math.radians(approach)
        h = math.radians(approach + r)
        return (x + da * math.cos(u) + db * math.cos(h),
                y + da * math.sin(u) + db * math.sin(h), heading)

    # ------------------------------------------------------------ 내부
    def _log(self, kind, detail=''):
        self.event_seq += 1
        self.events.append({'t': round(self._now, 2), 'event': kind, 'detail': detail})

    def _reset_primitive(self):
        self._ref_odo = self._odo
        self._ref_yaw = self._yaw_acc
        self._hold_yaw = None if self._pose is None else self._pose[2]
        self._hits = 0
        self._lost_since = None
        self._prev_error = None
        self._prev_t = None
        self._min_ok = 0.0
        self._max_ok = 0.0
        self._prim = {}
        self._tee_seen = {0: None, 2: None}   # T자: 왼쪽·오른쪽 센서가 마지막으로 선을 본 이동 거리
        # 옆 IR run: on = 지금 켜진 run 안, off = 연속 꺼진 샘플 수, valid = 창 시작 뒤 꺼짐→켜짐으로 시작한 run
        self._side_run = {ch: {'on': False, 'off': 0, 'valid': False, 'start': None, 'noted': False}
                          for ch in (0, 2)}

    def _update_odom(self, pose):
        if self._pose is not None:
            self._odo += math.hypot(pose[0] - self._pose[0], pose[1] - self._pose[1])
            self._yaw_acc += _wrap(pose[2] - self._pose[2])
        self._pose = pose
        if self._hold_yaw is None:
            self._hold_yaw = pose[2]

    def _finish(self, state, reason=''):
        self._gap = None
        self._recovery = None
        self._paused_from = None
        self.state = state
        self.reason = reason
        self._steps.clear()
        self._zero_once = True
        self._pre = None
        self._end = (self._end_of_leg(self.legs[self.index])
                     if state in (ARRIVED, HANDOFF_STATE) else None)
        if self._end is not None:
            self._end['pose'] = self._pose     # 끝난 자세(이어 붙이기·끝 기억 보고의 이동 검사)
        self._log(state, reason)

    def _end_of_leg(self, leg):
        """leg 도착 동작 뒤 노드·로봇이 보는 방향·출발 노드 → 센서 거리(이어서 출발할 때)."""
        c = self.cfg
        back = {'E': 'W', 'W': 'E', 'N': 'S', 'S': 'N'}[leg.heading]
        if leg.action == LOAD:    # 가로선 앞에서 멈춘 뒤 축 기준 약 180°: 센서가 노드 바깥쪽으로
            # 도킹 ①(a)은 상차 뒤 −a 로 되돌아오므로 기본 정지 자세와 같음
            return {'node': leg.to, 'facing': back, 's0': 2 * c.ir_ahead_cm + c.half_line_cm}
        if leg.action == PARK:    # 후진해 가로선 가장자리에서 멈춤
            return {'node': leg.to, 'facing': back, 's0': c.half_line_cm}
        return {'node': leg.to, 'facing': leg.heading, 's0': None}

    def _begin_leg(self, s0_cm, guess=False):
        """구간 시작. s0_cm = 출발 노드에서 센서까지 거리(모르면 None → 최소 거리 제한 없음).

        guess: s0 가 가정값(사람이 놓은 출발 위치)이면 True → 비율 창만 쓰고 gate_early 는 안 씀.
        """
        leg = self.legs[self.index]
        c = self.cfg
        # 노드 회전(ROTATE)이 끝나 바로 이 구간을 시작하면 처음 post_turn_slow_cm 은 감속
        self._slow_until = c.post_turn_slow_cm if self.state == ROTATE else 0.0
        self.state = FOLLOW
        self._reset_primitive()
        self._gap = None
        self._recovery = None
        self._searched = False
        self._yaw_hist.clear()
        self._leg_s0 = 0.0 if s0_cm is None else s0_cm
        self._progress = 0.0
        c = self.cfg
        target = leg.length - c.half_line_cm      # 센서가 도착 노드 선의 가까운 가장자리에 닿는 거리
        if s0_cm is None:
            self._expected = None
            self._min_ok = 0.0
            self._max_ok = c.gate_max * leg.length + c.gate_slack_cm
        else:
            expected = max(0.0, target - s0_cm)
            self._expected = expected
            self._min_ok = max(0.0, c.gate_min * expected - c.gate_slack_cm)
            if c.gate_early_cm > 0 and not guess:
                self._min_ok = max(self._min_ok, expected - c.gate_early_cm)
            self._max_ok = c.gate_max * expected + c.gate_slack_cm
        self._leg_ref = self._odo
        self._leg_guess = s0_cm is None or guess
        self._leg_begun = self.index
        self._psi_hist.clear()
        self._arrive_info = None
        self._log('leg', f'{leg.frm}→{leg.to} {leg.action}')

    def _track_sides(self, reading, d):
        """추종(GAP 포함) 샘플마다 옆 IR run 을 갱신. 창 시작 전 run 은 valid 가 아님."""
        k = self.cfg.side_rise_off_samples
        for ch in (0, 2):
            r = self._side_run[ch]
            if reading.on_line[ch]:
                if not r['on']:
                    r.update(on=True, valid=r['off'] >= k and d >= self._min_ok, start=d,
                             noted=False)
                r['off'] = 0
            else:
                r['off'] += 1
                if r['off'] >= k:
                    r['on'] = False
                    r['valid'] = False

    def _side_ok(self, ch, leg, d, tee=False):
        """옆 IR ch 를 노드 판정에 써도 되는지(켜짐 확인 규칙)."""
        c = self.cfg
        if not c.side_rise_only or (tee and not c.side_rise_tee):
            return True
        r = self._side_run[ch]
        if r['valid']:
            return True
        if r['on'] and not r['noted']:      # 창 전부터 켜져 있던 run: 한 번만 기록
            r['noted'] = True
            self._log('side_ignored', f'{leg.to} {"left" if ch == 0 else "right"} on since '
                      f'{r["start"]:.1f} cm (window {self._min_ok:.1f}), at {d:.1f} cm')
        return False

    def _hit(self, reading, leg, d=0.0):
        if leg.detect == 'bar':
            return reading.node
        on = reading.on_line
        if not on[1]:
            return False
        if self.cfg.tee_both and LEFT in leg.sides and RIGHT in leg.sides:
            seen = [ch for ch in (0, 2) if on[ch] and self._side_ok(ch, leg, d, tee=True)]
            for ch in seen:
                self._tee_seen[ch] = d
            a, b = self._tee_seen[0], self._tee_seen[2]
            return bool(seen) and a is not None and b is not None \
                and abs(a - b) <= self.cfg.tee_window_cm
        return ((LEFT in leg.sides and on[0] and self._side_ok(0, leg, d))
                or (RIGHT in leg.sides and on[2] and self._side_ok(2, leg, d)))

    def _follow(self, reading):
        c = self.cfg
        leg = self.legs[self.index]
        d = self._odo - self._ref_odo
        self._track_sides(reading, d)
        if self._gap is not None:
            cmd = self._gap_step(reading, leg, d)
            if self._gap is not None or self.state != FOLLOW:
                return cmd
            # 선을 다시 잡음: 이 샘플부터 평소 추종(감지 창 전이라 노드 판정은 아직 안 걸림)
        elif reading.lost and c.gap_max_cm > 0 and d >= c.suppress_cm:
            if d > self._max_ok:          # 예상 거리를 이미 넘음: 예전처럼 노드 놓침(GAP 하지 않음)
                self.fail(f'node_missed: {leg.to} not found within {self._max_ok:.1f} cm')
                return None
            return self._gap_start(leg, d)
        if d >= c.suppress_cm and d >= self._min_ok and self._hit(reading, leg, d):
            self._hits += 1
        else:
            self._hits = 0
        if self._hits >= c.node_confirm:
            self._arrive(leg)
            return self._dispatch(reading)
        if d > self._max_ok:
            self.fail(f'node_missed: {leg.to} not found within {self._max_ok:.1f} cm')
            return None
        if reading.lost:
            if self._lost_since is None:
                self._lost_since = self._now
            elif self._now - self._lost_since > c.lost_timeout:
                self.fail(f'line_lost on {leg.frm}→{leg.to}')
                return None
            self._prev_error = None
            return Command(0.0, 0.0)
        self._lost_since = None
        self._remember_yaw(d, reading)
        self._remember_psi(d, reading, leg)
        e = reading.error
        de = 0.0
        if self._prev_error is not None and self._prev_t is not None and self._now > self._prev_t:
            de = (e - self._prev_error) / (self._now - self._prev_t)
        self._prev_error, self._prev_t = e, self._now
        # 예상 노드 위치 approach_lead_cm 전부터 감속(출발 위치 모름 = 처음부터). 직진 통과 구간은 감속 안 함
        if c.pass_full_speed and leg.action == STRAIGHT:
            speed = c.base_speed
        elif self._expected is None:
            speed = self.approach_speed
        else:
            speed = self.approach_speed if d >= self._expected - c.approach_lead_cm else c.base_speed
        if d < self._slow_until:         # 회전 직후: 선 가운데로 돌아올 때까지 감속(가장자리 타기 방지)
            speed = min(speed, self.approach_speed)
        angular = _clamp(-(self.kp_for(speed) * e + c.kd * de), c.max_angular)
        linear = speed * (1.0 - c.slow_gain * abs(e))
        linear = min(c.max_linear, max(c.min_speed, linear))
        return Command(linear, angular)

    def _remember_yaw(self, d, reading):
        """선 위 추종 샘플: odom 방향과 선 위치 추정점을 최근 gap_heading_window_cm 만큼만 기억.

        선 위치 = 가운데 IR 위치 − 왼쪽 법선 × (오차 × IR 간격). 오차 +1 = 선이 오른쪽 IR 아래.
        """
        c = self.cfg
        x, y, yaw = self._pose
        sx, sy = x + c.ir_ahead_cm * math.cos(yaw), y + c.ir_ahead_cm * math.sin(yaw)
        off = -reading.error * c.ir_spacing_cm
        h = self._yaw_hist
        h.append((d, yaw, sx - off * math.sin(yaw), sy + off * math.cos(yaw)))
        while h and h[0][0] < d - c.gap_heading_window_cm:
            h.popleft()

    def _gap_heading(self):
        """GAP 방향. 기억한 거리가 GAP_CHORD_MIN_CM 이상이면 선 위치 점들의 앞·뒤 절반 평균을 이은 방향
        (회전 직후처럼 로봇이 선과 비스듬히 추종 중이어도 선 방향), 짧으면 odom 방향 평균."""
        h = list(self._yaw_hist)
        if not h:
            return self._pose[2]
        if h[-1][0] - h[0][0] >= GAP_CHORD_MIN_CM and len(h) >= 4:
            k = len(h) // 2
            ax = sum(p[2] for p in h[:k]) / k
            ay = sum(p[3] for p in h[:k]) / k
            bx = sum(p[2] for p in h[k:]) / (len(h) - k)
            by = sum(p[3] for p in h[k:]) / (len(h) - k)
            return math.atan2(by - ay, bx - ax)
        return math.atan2(sum(math.sin(p[1]) for p in h), sum(math.cos(p[1]) for p in h))

    def _gap_start(self, leg, d):
        yaw = self._gap_heading()
        self._gap = {'d': d, 'yaw': yaw, 'pose': self._pose[:2], 'hits': 0, 'window': False,
                     't0': self._now, 'prog_d': d, 'prog_t': self._now}
        self._prev_error = None
        self._lost_since = None
        self._hits = 0
        self._log('gap', f'{leg.frm}→{leg.to} at {d:.1f} cm, heading {math.degrees(yaw):.1f}°')
        return self._gap_cmd()

    def _gap_cmd(self):
        c = self.cfg
        w = _clamp(c.heading_kp * _wrap(self._gap['yaw'] - self._pose[2]), c.max_angular)
        return Command(self.approach_speed, w)

    def _gap_guard(self, leg, d):
        """GAP 시간·진행 감시. 넘으면 정지하고 True.

        시간 상한 = 2 + 2·gap_max_cm / min_speed(s, 도킹 이동 timeout 과 같은 꼴, 기본 약 12.7 s).
        진행 없음 = odom 이동이 GAP_PROGRESS_CM 미만인 채 lost_timeout 이 지남(odom 은 오는데 값이 그대로 등).
        """
        c = self.cfg
        g = self._gap
        if d - g['prog_d'] >= GAP_PROGRESS_CM:
            g['prog_d'], g['prog_t'] = d, self._now
        elif self._now - g['prog_t'] > c.lost_timeout:
            self.fail(f'line_lost: gap no progress ({d - g["d"]:.1f} cm) on {leg.frm}→{leg.to}')
            return True
        if self._now - g['t0'] > 2.0 + 2.0 * c.gap_max_cm / (c.min_speed * 100.0):
            self.fail(f'line_lost: gap timeout ({d - g["d"]:.1f} cm) on {leg.frm}→{leg.to}')
            return True
        return False

    def _gap_recover(self, reading, leg, d, where=''):
        """IR 하나라도 gap_confirm 샘플 연속 선 위면 추종으로(옆 IR 이면 추종 PD 가 조향). → 복귀했으면 True."""
        g = self._gap
        g['hits'] = g['hits'] + 1 if not reading.lost else 0
        if g['hits'] < self.cfg.gap_confirm:
            return False
        self._log('gap_recovered', f'{d - g["d"]:.1f} cm{where}')
        self._gap = None
        self._prev_error = None
        self._prev_t = None
        self._lost_since = None
        self._hits = 0
        return True

    def _gap_step(self, reading, leg, d):
        """GAP 한 샘플. self._gap 이 None 이 되고 상태가 follow 면 선을 다시 잡은 것(호출부가 추종을 이어 감)."""
        c = self.cfg
        g = self._gap
        if self._gap_guard(leg, d):
            return None
        if d >= self._min_ok:
            return self._gap_window_step(reading, leg, d)
        if self._gap_recover(reading, leg, d):
            return None
        if d - g['d'] > c.gap_max_cm:
            if c.search_deg <= 0:
                self.fail(f'line_lost: gap {d - g["d"]:.1f} cm exceeded {c.gap_max_cm:g} cm '
                          f'on {leg.frm}→{leg.to}')
                return None
            return self._search_start(reading, leg, d)
        return self._gap_cmd()

    def _gap_window_step(self, reading, leg, d):
        """감지 창 안 GAP: 같은 방향·속도로 직진하며 그 노드의 기존 판정으로만 노드를 찾음.

        진입로 테이프가 끊겨도 노드 테이프에 닿으면 평소처럼 도착. 창 안에서는 SEARCH(후진·회전)를 하지 않음
        (노드 근처에서 다른 선을 잡을 수 있어서). 노드 판정(_hit)을 먼저 보고, 노드가 아닌데 IR 이 선을 보면
        창 전과 같은 규칙으로 추종에 돌아감(추종의 노드 판정은 그대로라 판정 완화가 아님).
        """
        c = self.cfg
        g = self._gap
        if not g['window']:
            g['window'] = True
            self._log('gap_window', f'{leg.to} at {d:.1f} cm')
        if self._hit(reading, leg, d):
            self._hits += 1
        else:
            self._hits = 0
        if self._hits >= c.node_confirm:
            self._log('gap_node', f'{leg.to} after {d - g["d"]:.1f} cm')
            self._gap = None
            self._arrive(leg)
            return self._dispatch(reading)
        if d > self._max_ok or d - g['d'] > c.gap_max_cm:
            self.fail(f'line_lost: gap into {leg.to} window, node not found on {leg.frm}→{leg.to}')
            return None
        if self._gap_recover(reading, leg, d, f' (in {leg.to} window)'):
            return None
        return self._gap_cmd()

    def _search_start(self, reading, leg, d):
        """GAP 거리 초과: 놓친 지점으로 후진 → 좌·우·원래 방향으로 돌며 선 찾기. 구간마다 한 번."""
        c = self.cfg
        g = self._gap
        self._gap = None
        if self._searched:
            self.fail(f'line_lost: gap/search failed on {leg.frm}→{leg.to} (second gap after search)')
            return None
        self._searched = True
        self._recovery = {'d': g['d'], 'window': (self._min_ok, self._max_ok), 'yaw': g['yaw']}
        self._log('search', f'{leg.frm}→{leg.to} gap {d - g["d"]:.1f} cm > {c.gap_max_cm:g}')
        s = c.search_deg
        self._steps.clear()
        self._steps.extend([(ADVANCE, 0.0, 'search', g['pose']),
                            (ROTATE, 1.0, 0.0, s, 'search'),
                            (ROTATE, -1.0, 0.0, 2 * s, 'search'),
                            (ROTATE, 1.0, 0.0, s, 'search'),
                            ('search_fail',)])
        self._next_step()
        return self._dispatch(reading)

    def _search_found(self, turned):
        """SEARCH 회전 중 선 정렬: 남은 회전을 버리고 놓친 지점 이동 거리로 추종을 이어 감."""
        rec = self._recovery
        leg = self.legs[self.index]
        self._steps.clear()
        self._log('search_found', f'{leg.frm}→{leg.to} {math.degrees(turned):.0f}°')
        self.state = FOLLOW
        self._reset_primitive()
        self._min_ok, self._max_ok = rec['window']
        self._ref_odo = self._odo - rec['d']      # 후진·회전으로 늘어난 누적 거리를 빼고 놓친 지점부터
        self._leg_ref = self._ref_odo
        self._recovery = None
        self._yaw_hist.clear()

    def kp_for(self, speed):
        """직선 추종 kp. kp_speed_scale 이면 gain_ref_speed 를 넘는 속도에서 속도에 비례해 키움."""
        c = self.cfg
        if not c.kp_speed_scale or speed <= c.gain_ref_speed:
            return c.kp
        return c.kp * speed / c.gain_ref_speed

    def _arrive(self, leg):
        c = self.cfg
        self._save_leg_psi(self.index, leg)
        self._arrive_info = {'index': self.index, 'pose': self._pose,
                             'pos': self._leg_s0 + (self._odo - self._ref_odo)}
        self._log('node', f'{leg.to} ({leg.action})')
        steps = self._steps
        steps.clear()
        last = self.index == len(self.legs) - 1
        done = ('finish',) if last else ('via',)      # 경유 정차면 다음 구간으로
        if leg.action == STRAIGHT:
            steps.append((PASS, c.pass_cm))
            steps.append(('next', c.pass_cm - c.half_line_cm))
        elif leg.action in (LEFT, RIGHT):
            direction = 1.0 if leg.action == LEFT else -1.0
            steps.append((ADVANCE, c.turn_advance_cm))
            steps.append((ROTATE, direction, c.turn_min_deg, c.turn_max_deg, True))
            steps.append(('next', c.ir_ahead_cm))
        elif leg.action == LOAD:
            # (a 이동) → 반시계 r° → (b 이동) → 상차 대기 → (−b 이동) → 반시계로 진입선을 다시 찾음(합계 약 180°)
            # 기본 (0, 90, 0) 이면 반시계 90° → 상차 대기 → 반시계 turn_min~turn_max(예전과 같음)
            dv = self.dock.get(leg.to)
            r = dv.rotate_deg
            self._dock_disp = (0.0, 0.0)
            if dv.advance_cm:
                steps.append((ADVANCE, dv.advance_cm, 'a'))
            steps.append((ROTATE, 1.0, r, r, False, 'r'))
            if dv.move_cm:
                steps.append((ADVANCE, dv.move_cm, 'b'))
            steps.append((LOADING,))
            if dv.move_cm:
                steps.append((ADVANCE, -dv.move_cm, '-b'))
            if dv.advance_cm:
                # a ≠ 0(2026-11-09 사용자 결정 (나)): 시계 r° 로 접근 방향을 다시 보고 −a 만큼 되돌아와 기본 정지
                # 자세로 → 반시계 90 + turn_min ~ 90 + turn_max(기본값 경로의 합계)로 진입선 찾기.
                # a > +4.5 cm 그대로 돌아보면 IR 이 가로선 위에 남아 진입선을 구분 못 함(시뮬 a +6 에서 22/24 정지).
                steps.append((ROTATE, -1.0, r, r, False, 'unr'))
                steps.append((ADVANCE, -dv.advance_cm, '-a'))
                lo, hi = 90.0 + c.turn_min_deg, 90.0 + c.turn_max_deg
                phase = 'back_a'
            elif r == 90.0:
                lo, hi = c.turn_min_deg, c.turn_max_deg
                phase = 'back'
            else:     # 남은 각 (180 − r) 둘레로 기본과 같은 폭(기본 −30°·+30°)
                lo, hi = (180.0 - r) + (c.turn_min_deg - 90.0), (180.0 - r) + (c.turn_max_deg - 90.0)
                phase = 'back'
            steps.append((ROTATE, 1.0, lo, hi, True, phase))
            steps.append(done + (ARRIVED,))
        elif leg.action == PARK:
            steps.append((ROTATE, 1.0, c.back_min_deg, c.back_max_deg, True))
            steps.append((REVERSE, c.reverse_max_cm))
            steps.append(done + (ARRIVED,))
        elif leg.action == HANDOFF:
            steps.append(done + (HANDOFF_STATE,))
        elif leg.action == STOP:
            steps.append(done + (ARRIVED,))
        else:  # pragma: no cover - plan_route 가 막음
            raise GraphError(f'unknown action {leg.action}')
        if self.index not in self._seg_ends and leg.action not in (STRAIGHT, LEFT, RIGHT):
            raise GraphError('arrival action before the last leg')  # pragma: no cover
        self._next_step()

    def _next_step(self):
        if not self._steps:
            self._finish(ARRIVED, 'route done')
            return
        st = self._steps.popleft()
        kind = st[0]
        if kind == 'next':
            self.index += 1
            self._begin_leg(st[1])
            return
        if kind == 'finish':
            self._finish(st[1], 'route done')
            return
        if kind == 'via':        # 경유 정차 동작이 끝남 → 이어서 출발
            leg = self.legs[self.index]
            self._log('via', leg.to)
            end = self._end_of_leg(leg)
            self.index += 1
            self._loaded = False
            self._dock_phase = None
            self._depart(self.legs[self.index], end)
            return
        if kind == 'begin':      # 출발 전 회전이 끝남 → 첫 구간
            self._pre = None
            self._begin_leg(st[1])
            return
        if kind == 'rv_line':
            self._rv_line_begin(st[1])
            return
        if kind == 'rv_center':
            self._rv_center()
            return
        if kind == 'rv_align':
            self._rv_align(st[1])
            return
        if kind == 'rv_done':
            self._rv_done(st[1])
            return
        if kind == 'search_fail':
            leg = self.legs[self.index]
            self._log('search_failed', f'{leg.frm}→{leg.to}')
            self.fail(f'line_lost: gap/search failed on {leg.frm}→{leg.to}')
            return
        self._dock_step_done()
        self.state = kind
        self._reset_primitive()
        if kind == ADVANCE and len(st) > 2:      # 도킹·SEARCH 이동: 부호 있는 거리, 멈춘 뒤 시작
            self._prim = {'cm': st[1], 'signed': True, 'phase': st[2], 'settled': False,
                          'still': 0, 'last': None, 'last_stamp': None, 't0': self._now,
                          'proj': 0.0, 'to': st[3] if len(st) > 3 else None}
            if str(st[2]).startswith('rv'):        # 라인 후진의 짧은 이동: 후진 속도 이하
                self._prim['vmax'] = self._rv_speed()
                self._rv_phase(st[2])
            elif st[2] != 'search':
                self._dock_phase = st[2]
        elif kind in (PASS, ADVANCE):
            self._prim = {'cm': st[1]}
        elif kind == ROTATE:
            self._prim = {'dir': st[1], 'min': math.radians(st[2]), 'max': math.radians(st[3]),
                          'acquire': st[4]}
            if len(st) > 5 and str(st[5]).startswith('rv'):
                self._rv_phase(st[5])
            elif len(st) > 5:
                self._dock_phase = st[5]
        elif kind == REVERSE:
            self._prim = {'cm': st[1]}
        elif kind == LOADING and self._dock_phase is not None:
            self._dock_phase = 'load'
        self._log(kind, '' if kind == LOADING else str(st[1:]))

    def _dock_step_done(self):
        """끝난 도킹 이동을 표시용 누적값에 더한다."""
        p = self._prim
        if p.get('signed') and p['phase'] != 'search' and not str(p['phase']).startswith('rv') \
                and not p.get('counted'):
            p['counted'] = True
            a, b = self._dock_disp
            if p['phase'] in ('a', '-a'):
                a += p['proj']
            else:
                b += p['proj']
            self._dock_disp = (a, b)

    def _dispatch(self, reading):
        """상태가 바뀐 같은 샘플에서 새 상태의 명령을 낸다."""
        if self.state in TERMINAL:
            return None
        if self.state == FOLLOW:
            return Command(0.0, 0.0)  # 다음 샘플부터 추종
        return {PASS: self._drive, ADVANCE: self._drive, ROTATE: self._rotate,
                LOADING: self._loading, REVERSE: self._reverse,
                PAUSED: self._paused}[self.state](reading)

    def _hold(self):
        return _clamp(self.cfg.heading_kp * _wrap(self._hold_yaw - self._pose[2]),
                      self.cfg.max_angular)

    def _drive(self, reading):
        if self._prim.get('signed'):
            return self._dock_move(reading)
        if self._odo - self._ref_odo >= self._prim['cm']:
            self._next_step()
            return self._dispatch(reading)
        fast = self.state == PASS and self.cfg.pass_full_speed     # 직진 통과는 감속 안 함
        return Command(self.cfg.base_speed if fast else self.approach_speed, self._hold())

    def _dock_move(self, reading):
        """도킹 이동: 0 명령으로 odom 이 멈추길 기다린 뒤, 그 자세에서 부호 있는 거리(방향 유지·감속)."""
        c = self.cfg
        p = self._prim
        x, y, yaw = self._pose
        if not p['settled']:
            stamp = self._pose_stamp
            fresh = stamp is None or stamp != p['last_stamp']
            if p['last'] is None:            # 첫 샘플은 비교 기준만(같은 pose 로 멈춤을 세지 않음)
                p['last'], p['last_stamp'] = self._pose, stamp
            elif fresh:                      # odom 이 새로 들어온 샘플끼리만 비교
                lx, ly, lyaw = p['last']
                still = (math.hypot(x - lx, y - ly) < SETTLE_CM
                         and abs(_wrap(yaw - lyaw)) < math.radians(SETTLE_DEG))
                p['last'], p['last_stamp'] = self._pose, stamp
                p['still'] = p['still'] + 1 if still else 0
            if p['still'] >= c.node_confirm:
                p.update(settled=True, origin=(x, y), t0=self._now)
                self._hold_yaw = yaw
                if p['to'] is not None:      # SEARCH: 멈춘 자리에서 놓친 지점까지(방향 투영, 보통 음수)
                    tx, ty = p['to']
                    p['cm'] = (tx - x) * math.cos(yaw) + (ty - y) * math.sin(yaw)
                    limit = c.ir_ahead_cm - c.half_line_cm + PRE_CENTER_MARGIN_CM
                    if p['phase'] == 'rv_pre' and abs(p['cm']) > limit:
                        self.fail(f'pre_center: move {p["cm"]:+.1f} cm exceeds {limit:.1f} cm')
                        return None
                    if abs(p['cm']) < 0.05:
                        self._next_step()
                        return self._dispatch(reading)
            elif self._now - p['t0'] > SETTLE_TIMEOUT_S:
                self.fail(f'{self._move_label()}: odom did not settle')
                return None
            return Command(0.0, 0.0)
        ox, oy = p['origin']
        proj = (x - ox) * math.cos(self._hold_yaw) + (y - oy) * math.sin(self._hold_yaw)
        p['proj'] = proj
        target = p['cm']
        sign = 1.0 if target > 0 else -1.0
        remaining = (target - proj) * sign
        if remaining <= 0.0:
            self._next_step()
            return self._dispatch(reading)
        # 반대쪽으로 밀리거나(remaining 이 목표 + 여유보다 큼) 경로 길이가 넘치면 정지
        if remaining > abs(target) + MOVE_OVERRUN_CM or \
                self._odo - self._ref_odo > abs(target) + 2 * MOVE_OVERRUN_CM:
            self.fail(f'{self._move_label()} overrun ({proj:.1f} of {target:.1f} cm)')
            return None
        if self._now - p['t0'] > 2.0 + 2.0 * abs(target) / (c.min_speed * 100.0):
            self.fail(f'{self._move_label()} timeout ({proj:.1f} of {target:.1f} cm)')
            return None
        speed = min(self.approach_speed, max(c.min_speed, MOVE_GAIN * remaining))
        if 'vmax' in p:
            speed = min(speed, p['vmax'])
        return Command(sign * speed, self._hold())

    def _move_label(self):
        phase = str(self._prim.get('phase'))
        if phase.startswith('rv'):
            return 'reverse_move'
        return 'gap_search back' if phase == 'search' else 'dock_move'

    def _rotate(self, reading):
        c = self.cfg
        p = self._prim
        turned = abs(self._yaw_acc - self._ref_yaw)
        if p['acquire']:
            if (turned >= p['min'] and reading.on_line[1]
                    and abs(reading.error) <= c.align_error):
                self._hits += 1
            else:
                self._hits = 0
            if self._hits >= c.node_confirm:
                if p['acquire'] == 'search':
                    self._search_found(turned)
                    return Command(0.0, 0.0)
                self._log('line_found', f'{math.degrees(turned):.0f}°')
                self._next_step()
                return self._dispatch(reading)
            if turned > p['max']:
                if p['acquire'] == 'search':     # 이 방향에 없음 → 다음 SEARCH 회전
                    self._next_step()
                    return self._dispatch(reading)
                self.fail(f'line_not_found after {math.degrees(turned):.0f}°')
                return None
            if turned < p['min']:
                speed = c.rotate_speed
            else:   # 찾기 시작하면 천천히(정렬 판정을 놓치지 않게)
                speed = max(c.rotate_min_speed, 0.6 * c.rotate_speed)
            return Command(0.0, p['dir'] * speed)
        remaining = p['max'] - turned
        if remaining <= math.radians(1.5):
            self._next_step()
            return self._dispatch(reading)
        speed = min(c.rotate_speed, max(c.rotate_min_speed, 1.5 * remaining))
        return Command(0.0, p['dir'] * speed)

    def _loading(self, reading):
        return Command(0.0, 0.0)

    def _reverse(self, reading):
        if self._prim.get('mode') == 'line':      # 라인 후진(FFT1-172)
            return self._reverse_follow(reading)
        c = self.cfg
        if reading.node:
            self._hits += 1
        else:
            self._hits = 0
        if self._hits >= c.node_confirm:
            self._next_step()
            return self._dispatch(reading)
        if self._odo - self._ref_odo > self._prim['cm']:
            self.fail(f'park_overrun: bar not found within {self._prim["cm"]:.0f} cm')
            return None
        return Command(-min(c.reverse_speed, self.approach_speed), self._hold())

    # ------------------------------------------------------------ 일시정지·라인 후진 내부(FFT1-172)
    def _paused(self, reading):
        if self._paused_from == FOLLOW and self._gap is None and reading is not None:
            self._track_sides(reading, self._odo - self._ref_odo)   # 멈춘 동안 옆 IR run 기록만
        if not self._pause_noted and self.settled:
            self._pause_noted = True
            self._log('paused_settled', self._paused_from)
        return Command(0.0, 0.0)

    def _shift_time(self, dt):
        """일시정지한 시간만큼 시간 기준(시간 초과·놓침 시작)을 민다."""
        if self._lost_since is not None:
            self._lost_since += dt
        for k in ('t0', 'lost_t'):
            if self._prim.get(k) is not None:
                self._prim[k] += dt
        if self._gap is not None:
            self._gap['t0'] += dt
            self._gap['prog_t'] += dt

    def _watch_still(self, pose, now, stamp):
        """odom 정지 확인(모든 상태에서 기록만). 같은 odom(stamp 같음)은 건너뜀."""
        if pose is None:
            self._still_last = self._still_since = None
            self._still_hist.clear()
            return
        t = now if stamp is None else stamp
        last = self._still_last
        if last is not None and t - last[0] <= 1e-6:
            return
        hist = self._still_hist
        if last is None:
            hist.clear()
        hist.append((t, pose))
        while len(hist) > 2 and hist[1][0] <= t - STILL_S:   # 창 시작보다 오래된 것 하나만 남김
            hist.popleft()
        self._still_last = (t, pose)
        # 창 안 모든 자세가 지금 자세에서 STILL_POS_CM·STILL_YAW_RAD 안이면 창 시작부터 멈춘 것으로 본다
        still = all(math.hypot(pose[0] - p[0], pose[1] - p[1]) < STILL_POS_CM
                    and abs(_wrap(pose[2] - p[2])) < STILL_YAW_RAD for _, p in hist)
        self._still_since = hist[0][0] if still and len(hist) > 1 else None

    def _remember_psi(self, d, reading, leg):
        """후진용 선 방향 샘플: 구간 시작 suppress_cm 뒤 ~ 감지 창 시작 전(노드 옆선이 오차를 치우치지 않는 곳)."""
        c = self.cfg
        if reading.node or d < c.suppress_cm or not self._yaw_hist:
            return
        cut = self._min_ok if self._expected is not None else \
            c.gate_min * leg.length - c.gate_slack_cm
        if d >= cut:
            return
        _, _, lx, ly = self._yaw_hist[-1]
        h = self._psi_hist
        h.append((d, lx, ly))
        while h and h[0][0] < d - PSI_WINDOW_CM:
            h.popleft()

    @staticmethod
    def _line_dir(h):
        """선 위치 점 (d, x, y) 의 앞·뒤 절반 평균을 이은 방향(_gap_heading 과 같은 방식). 2 cm 미만이면 None."""
        if len(h) < 4 or h[-1][0] - h[0][0] < GAP_CHORD_MIN_CM:
            return None
        k = len(h) // 2
        ax = sum(p[1] for p in list(h)[:k]) / k
        ay = sum(p[2] for p in list(h)[:k]) / k
        bx = sum(p[1] for p in list(h)[k:]) / (len(h) - k)
        by = sum(p[2] for p in list(h)[k:]) / (len(h) - k)
        return math.atan2(by - ay, bx - ax)

    def _save_leg_psi(self, index, leg):
        v = self._line_dir(self._psi_hist)
        if v is not None:
            self._leg_psi[index] = v
            self._map_off = _wrap(v - math.radians(self._HEADING_DEG[leg.heading]))

    def _psi_of(self, k):
        """구간 k 의 선 방향(odom yaw). 기억 → (지금 구간) 선 위 샘플 → 지도 방향 + 다른 구간에서 구한 차이
        → (지금 구간) GAP 방향(odom 평균) 순. 모르면 None."""
        if k in self._leg_psi:
            return self._leg_psi[k]
        current = k == self.index and self._leg_begun == k
        if current:
            v = self._line_dir(self._psi_hist)
            if v is not None:
                return v
        if self._map_off is not None:
            return _wrap(math.radians(self._HEADING_DEG[self.legs[k].heading]) + self._map_off)
        if current and self._yaw_hist:
            return self._gap_heading()
        return None

    def _leg_pos(self):
        """지금 구간 출발 노드 → 센서 거리(cm, 구간 시작 위치 + 이동 거리)."""
        return self._leg_s0 + (self._odo - self._leg_ref)

    def _arrival_pos(self, k, pose=None):
        """구간 k 끝 노드 감지 때 센서 위치 + 그 뒤 축 이동(선 방향 투영). 모르면 None."""
        info = self._arrive_info
        pose = pose or self._pose
        if info is None or info['index'] != k or pose is None:
            return None
        psi = self._psi_of(k)
        if psi is None:
            psi = info['pose'][2]
        ax, ay = info['pose'][0], info['pose'][1]
        return info['pos'] + ((pose[0] - ax) * math.cos(psi)
                              + (pose[1] - ay) * math.sin(psi))

    def _rv_speed(self):
        return min(self.cfg.reverse_follow_speed, REVERSE_FOLLOW_MAX, self.cfg.max_linear)

    def _rv_phase(self, phase):
        if self._back is not None:
            self._back['phase'] = phase

    def _reverse_target(self, k):
        """구간 k 를 보고 선 채(방향 legs[k].heading) 후진해 legs[k].frm 을 찾을 때의 판정용 Leg. 못 찾으면 None."""
        leg = self.legs[k]
        nid = leg.frm
        n = self.graph.nodes[nid]
        if n.bar:
            return Leg(leg.to, nid, leg.heading, leg.length, 'reverse', 'bar', frozenset())
        sides = {turn_of(leg.heading, self.graph.heading(nid, o)) for o in n.links if o != leg.to}
        sides &= {LEFT, RIGHT}
        if not sides:
            return None
        return Leg(leg.to, nid, leg.heading, leg.length, 'reverse', 'side', frozenset(sides))

    def _undo_dock_steps(self, leg):
        """상차 대기 자세 → 가로선 정지 자세(③ −b → ② 시계 r° → ① −a). 기존 도킹 단계 이름을 써서 화면 추정도 같음."""
        dv = self.dock.get(leg.to)
        steps = []
        if dv.move_cm:
            steps.append((ADVANCE, -dv.move_cm, '-b'))
        steps.append((ROTATE, -1.0, dv.rotate_deg, dv.rotate_deg, False, 'unr'))
        if dv.advance_cm:
            steps.append((ADVANCE, -dv.advance_cm, '-a'))
        return steps

    def _node_xy(self, nid, heading, along):
        n = self.graph.nodes[nid]
        h = math.radians(self._HEADING_DEG[heading])
        return n.x + along * math.cos(h), n.y + along * math.sin(h)

    def _rv_line_begin(self, spec):
        """후진 한 구간 시작: 센서 위치를 정하고, 짧으면 odom 이동, 아니면 라인 후진(상태 피드백)."""
        c = self.cfg
        self._dock_step_done()        # (다′) 마지막 −a 이동을 도킹 표시 누적값에 더함
        self._dock_phase = None
        self._rv_det = None
        pos = spec['pos']
        if pos is None:
            pos = self._arrival_pos(spec['arrival'])
            if pos is None:
                self.fail('reverse_plan: position on the leg unknown')
                return
        total = pos + spec['extra']
        leg = self.legs[spec['k']]
        self.index = spec['k']
        if self._back is not None:
            self._back['leg'] = [leg.frm, self.legs[spec['k0']].to]
        if total < c.reverse_short_cm:
            # (가′) 출발 노드에서 가까움: 옆선 켜짐이 다시 안 생길 수 있어 판정 대신 odom 거리만큼.
            # 목표 = 센서가 노드 중심(끝) 또는 축이 노드 중심(회전 되돌리기 전). 구간 시작 위치는 감지를 테이프
            # 가장자리로 본 값이라 실제보다 약 0.5 cm 작음(2D 시뮬) → 반폭 대신 중심을 겨눠 테이프 위에 남게 함
            target = 0.0 if spec['final'] else c.ir_ahead_cm
            cm = -(total - target)
            self._log('reverse_short', f'{spec["node"]} {cm:+.1f} cm (sensor {total:.1f} cm)')
            ax = target - c.ir_ahead_cm
            x, y = self._node_xy(spec['node'], leg.heading, ax)
            if self._back is not None:
                self._back['est'] = (x, y, self._HEADING_DEG[leg.heading])
            if abs(cm) >= 0.05 and (not spec['final'] or cm < -0.3):
                self._steps.appendleft((ADVANCE, cm, 'rv_short'))
            self._next_step()
            return
        if spec['psi'] is None:
            self.fail('reverse_plan: line direction unknown')
            return
        expected = total - c.half_line_cm
        guess = spec['guess']
        self.state = REVERSE
        self._reset_primitive()
        self._min_ok = max(0.0, c.gate_min * expected - c.gate_slack_cm)
        if c.gate_early_cm > 0 and not guess:
            self._min_ok = max(self._min_ok, expected - c.gate_early_cm)
        self._max_ok = c.gate_max * expected + c.gate_slack_cm
        self._prim = {'mode': 'line', 'spec': spec, 'pos0': total, 'expected': expected,
                      'psi': spec['psi'], 'lost_d': None, 'lost_t': None, 't0': self._now,
                      'timeout': 2.0 + 2.0 * self._max_ok / (REVERSE_FOLLOW_MIN * 100.0),
                      'y': None, 'theta': 0.0, 'blind': [total - m for m in spec['marks']],
                      'acquire': spec.get('acquire_cm', 0.0)}
        self._rv_phase('reverse')
        if not spec['final']:
            self._steps.appendleft(('rv_center',))
        self._log('reverse', f'{leg.to}→{spec["node"]} expected {expected:.1f} cm '
                  f'window {self._min_ok:.1f}..{self._max_ok:.1f}'
                  f'{" (guess)" if guess else ""}')

    def _rv_align(self, psi):
        """후진 전 제자리 회전으로 odom 방향을 선 방향 ψ 에 맞춤(RV_ALIGN_DEG 이하면 건너뜀)."""
        self._dock_step_done()
        delta = _wrap(psi - self._pose[2])
        if abs(delta) > math.radians(RV_ALIGN_DEG):
            ang = math.degrees(abs(delta))
            self._steps.appendleft((ROTATE, 1.0 if delta > 0 else -1.0, ang, ang, False,
                                    'rv_align'))
        self._next_step()

    def _rv_center(self):
        """후진으로 노드 판정(센서 ≈ 노드 + half_line) → 축을 노드 중심으로(멈춘 뒤 판정 자세 기준 투영 거리)."""
        c = self.cfg
        x, y, yaw = self._rv_det
        k = c.ir_ahead_cm - c.half_line_cm
        to = (x + k * math.cos(yaw), y + k * math.sin(yaw))
        self._steps.appendleft((ADVANCE, 0.0, 'rv_center', to))
        self._next_step()

    def _rv_done(self, j):
        c = self.cfg
        leg = self.legs[j]
        nid = leg.frm
        x, y = self._node_xy(nid, leg.heading, c.half_line_cm - c.ir_ahead_cm)
        self._back.update(phase='done', est=(x, y, self._HEADING_DEG[leg.heading]))
        self._log('reverse_arrived', f'{nid} facing {leg.heading}')
        # 노드 중심(odom): 판정으로 멈췄으면 감지 때 축 + (ir_ahead − 반폭), odom 이동이면 센서 목표 = 중심
        if self._rv_det is not None:
            ox, oy, oyaw = self._rv_det
            k = c.ir_ahead_cm - c.half_line_cm
        else:
            ox, oy, oyaw = self._pose
            k = c.ir_ahead_cm
        center = (ox + k * math.cos(oyaw), oy + k * math.sin(oyaw))
        self.index = j
        self._finish(ARRIVED, f'reverse done at {nid}')
        # 끝 자세 = 그 노드에 이 방향으로 전진 도착해 감지한 자세(센서가 노드 위) → 이어서 출발 s0 = half_line.
        # center: 이어서 출발이 좌·우 회전이면 회전 전에 축을 여기로(전진 보정, 계약 2 의 끝 자세는 그대로)
        self._end = {'node': nid, 'facing': leg.heading, 's0': c.half_line_cm, 'center': center,
                     'pose': self._pose}
        # 경로를 지나온 앞부분으로 줄임: route[-1] = 지금 노드, leg = 그 노드로 들어온 구간(출발 노드면 없음)
        self.route = self.route[:j + 1]
        self.legs = self.legs[:j]
        self._base = 0                     # 줄인 경로 = 지나온 길 전체(이어 붙인 앞부분 포함)
        if self.legs and self.legs[-1].action not in (STRAIGHT, LEFT, RIGHT):
            # 경유 정차 노드(상차·주차)까지 되감음: 그 노드의 도착 동작은 하지 않았으므로 정차(stop)로 표시
            self.legs[-1] = replace(self.legs[-1], action=STOP)
        self._seg_ends = {k for k in self._seg_ends if k < j - 1} | ({j - 1} if j > 0 else set())
        self.index = max(0, j - 1)

    def _rv_fail(self, kind, detail):
        self._log(kind, detail)
        self.fail(f'{kind}: {detail}')

    def _reverse_omega(self, reading, y_hat, theta, speed):
        """후진 상태 피드백 ω = +k_y·ŷ − k_θ·θ̂ (k_y = ωₙ²/|v| per cm, k_θ = 2ζωₙ). ŷ None 이면 방향만."""
        c = self.cfg
        w = -2.0 * c.reverse_zeta * c.reverse_wn * theta
        if y_hat is not None:
            w += c.reverse_wn ** 2 / (speed * 100.0) * y_hat
        return w

    def _reverse_follow(self, reading):
        c = self.cfg
        p = self._prim
        spec = p['spec']
        target = spec['pseudo']
        d = self._odo - self._ref_odo
        self._track_sides(reading, d)
        if d >= c.suppress_cm and d >= self._min_ok and self._hit(reading, target, d):
            self._hits += 1
        else:
            self._hits = 0
        if self._hits >= c.node_confirm:
            self._rv_det = self._pose
            self._log('reverse_node', f'{target.to} at {d:.1f} cm (expected {p["expected"]:.1f})')
            if not spec['final'] and self._back is not None:
                x, y = self._node_xy(target.to, target.heading, c.half_line_cm - c.ir_ahead_cm)
                self._back['est'] = (x, y, self._HEADING_DEG[target.heading])
            self._next_step()
            return self._dispatch(reading)
        if d > self._max_ok:
            self._rv_fail('reverse_node_missed',
                          f'{target.to} not found within {self._max_ok:.1f} cm')
            return None
        if self._now - p['t0'] > p['timeout']:
            self._rv_fail('reverse_timeout', f'{target.to} at {d:.1f} cm')
            return None
        theta = _wrap(self._pose[2] - p['psi'])
        p['theta'] = theta
        if abs(theta) > math.radians(c.reverse_max_heading_deg):
            self._rv_fail('reverse_heading', f'{math.degrees(theta):.1f} deg at {d:.1f} cm')
            return None
        y = None
        if p['acquire'] and not reading.lost:
            p['acquire'] = 0.0                  # 선을 다시 잡음 → 이제부터 놓침을 셈
        if reading.lost and p['acquire']:
            # 직진 선 없는 노드를 되돌린 직후: 센서가 테이프 끝 너머에서 시작. 방향만 유지하며 acquire_cm 안에 잡아야 함
            if d > p['acquire']:
                self._rv_fail('reverse_lost', f'line not reacquired within {p["acquire"]:.1f} cm '
                              f'to {target.to}')
                return None
        elif reading.lost:
            if p['lost_d'] is None:
                p['lost_d'], p['lost_t'] = d, self._now
            elif (d - p['lost_d'] >= c.reverse_lost_cm or self._now - p['lost_t']
                  > c.lost_timeout + c.reverse_lost_cm / (REVERSE_FOLLOW_MIN * 100.0)):
                self._rv_fail('reverse_lost', f'{d - p["lost_d"]:.1f} cm at {d:.1f} cm '
                              f'to {target.to}')
                return None
        else:
            p['lost_d'] = p['lost_t'] = None
            y = reading.error * c.ir_spacing_cm - c.ir_ahead_cm * math.sin(theta)
        p['y'] = y
        # ŷ 를 쓰지 않는 곳(방향 θ̂ 만 유지): 구간 시작, 지나치는 노드 옆선 근처, 목적 노드 앞
        # max(창 시작, 예상 − gate_early_cm) 뒤·목적 노드 테이프를 밟은 뒤. 옆 IR 이 노드 선에 걸리기 시작하면
        # (판정 전에도 정규화 값이 먼저 올라) 오차가 치우쳐 마지막 샘플들에 차체를 돌림 — 2D 시뮬 끝 방향 1.3~1.9°
        near = d >= max(self._min_ok, p['expected'] - c.gate_early_cm)
        blind = (d < c.suppress_cm or near
                 or any(abs(d - b) < c.half_line_cm + REVERSE_BLIND_CM for b in p['blind'])
                 or (d >= self._min_ok and (reading.node or self._target_side_on(reading, target))))
        speed = self._rv_speed()
        w = self._reverse_omega(reading, None if blind else y, theta, speed)
        return Command(-speed, _clamp(w, c.max_angular))

    @staticmethod
    def _target_side_on(reading, target):
        on = reading.on_line
        return (LEFT in target.sides and on[0]) or (RIGHT in target.sides and on[2]) or \
            (target.detect == 'bar' and (on[0] or on[2]) and on[1])

    def _reverse_dict(self):
        b = self._back
        if b is None:
            return None
        out = {'to': b['to'], 'phase': b['phase'], 'from': b['from'], 'leg': b['leg'],
               'expected_cm': None, 'done_cm': None, 'y_cm': None, 'theta_deg': None,
               'window_cm': None}
        p = self._prim
        if self.state == REVERSE and p.get('mode') == 'line':
            out.update(expected_cm=round(p['expected'], 1),
                       done_cm=round(self._odo - self._ref_odo, 1),
                       y_cm=None if p['y'] is None else round(p['y'], 1),
                       theta_deg=round(math.degrees(p['theta']), 1),
                       window_cm=[round(self._min_ok, 1), round(self._max_ok, 1)])
        return out

    def _back_estimate(self):
        """후진 중·후진 도착 뒤 화면 표시용 바퀴 축 추정. 도킹 되돌림 중이면 None(기존 추정)."""
        b = self._back
        if b is None or b['phase'] == 'undo_dock':
            return None
        p = self._prim
        if self.state == REVERSE and p.get('mode') == 'line' and b['phase'] != 'done':
            spec = p['spec']
            leg = self.legs[spec['k']]
            axle = p['pos0'] - (self._odo - self._ref_odo) - self.cfg.ir_ahead_cm
            x, y = self._node_xy(spec['node'], leg.heading, axle)
            f = min(1.0, max(0.0, axle / leg.length)) if leg.length > 0 else 0.0
            return {'leg_progress': round(f, 3), 'pose_est': {
                'x_cm': round(x, 1), 'y_cm': round(y, 1),
                'heading_deg': round(self._HEADING_DEG[leg.heading], 1)}}
        if b['est'] is None:
            return None
        x, y, h = b['est']
        if self.state == ROTATE and p and b['phase'] != 'done':
            h += p['dir'] * math.degrees(abs(self._yaw_acc - self._ref_yaw))
        h = (h + 180.0) % 360.0 - 180.0
        return {'leg_progress': None, 'pose_est': {
            'x_cm': round(x, 1), 'y_cm': round(y, 1), 'heading_deg': round(h, 1)}}


__all__ = ['NavConfig', 'Command', 'RouteExecutor', 'GraphError', 'IDLE', 'FOLLOW', 'PASS',
           'ADVANCE', 'ROTATE', 'LOADING', 'REVERSE', 'ARRIVED', 'HANDOFF_STATE', 'STOPPED',
           'ERROR', 'STOP', 'TERMINAL', 'PAUSED', 'REVERSE_FOLLOW_MIN', 'REVERSE_FOLLOW_MAX']
