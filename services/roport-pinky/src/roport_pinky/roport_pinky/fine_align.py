"""마커 없는 마지막 정밀 접근(FineAlign). 기본 끔.

Nav2Bridge(trial_nav2)의 정밀 접근 단계(게이트 마커 GateDocking 과 같은 슬롯)를 그대로 쓴다:
Nav2 가 목표 20 cm 뒤까지 간 뒤 멈춘 것을 확인하고, 목표까지 직선 경로를 GateDock 제어기(충돌 검사 유지)로
보내 1.5 cm·0.08 rad 안에 들어왔는지 확인한다(trial_nav2 최종 확인). 마커 대신 명령의 목표 좌표를 쓴다.

왜: N12 복귀 자세에서 Nav2 가 일반 도착 허용 오차(8 cm) 안에서 멈춰, 라인 전환 조건(5 cm·가운데 IR 이 선 위)을
못 맞추고 매번 7 cm 앞에 섰다(11-22 d660 두 번). roport_smartcart real_return 의 N12 정밀 맞춤과 같은 생각이다.

켜는 방법: 드라이버 환경 변수 ROPORT_FINE_ALIGN_GOALS=쉼표로 구분한 goal_id(예: N12_RETURN), 또는 시험 실행기
(roport_follow demo_runner --fine-align, demo_RET_N12). ROPORT_GATE_DOCK_CONFIG 가 있으면 GateDocking 이 우선이다.
Nav2 설정에 GateDock 제어기·gate_dock_goal_checker 가 있어야 한다(trial.launch.py 가 둘 중 하나가 있으면 넣는다).
"""
import math
import os

ENV = 'ROPORT_FINE_ALIGN_GOALS'


def goals_from_env(environ=None):
    """ROPORT_FINE_ALIGN_GOALS → goal_id 튜플. 공백을 지우고 빈 항목은 버린다. 없거나 비면 ()."""
    raw = (os.environ if environ is None else environ).get(ENV) or ''
    return tuple(goal for goal in (part.strip() for part in raw.split(',')) if goal)


class FineAlign:
    """GateDocking 과 같은 함수 이름(reset·enabled·begin·target·active)을 가진다."""

    MAX_OFFSET_M = 0.30         # 정밀 접근을 시작할 때 목표까지 허용 거리(20 cm 뒤 + Nav2 도착 오차)
    MAX_YAW_RAD = 0.35
    TIMEOUT_S = 25.0

    def __init__(self, goals):
        self.goals = frozenset(goals)
        self.active, self.command, self.started = False, None, None

    def add_goals(self, goals):
        self.goals = self.goals | frozenset(goals)

    def reset(self, command=None):
        self.active, self.command, self.started = False, command, None

    def enabled(self, goal_id):
        return goal_id in self.goals

    def begin(self, now):
        self.active, self.started = True, now

    def target(self, pose, now):
        if self.command is None:
            raise ValueError('fine_align_no_goal')
        if self.started is not None and now - self.started > self.TIMEOUT_S:
            raise ValueError('fine_align_timeout')
        goal = self.command['pose']
        target = {'x': float(goal['x']), 'y': float(goal['y']), 'yaw': float(goal['yaw'])}
        yaw_error = math.atan2(math.sin(target['yaw'] - pose['yaw']), math.cos(target['yaw'] - pose['yaw']))
        if (math.hypot(target['x'] - pose['x'], target['y'] - pose['y']) > self.MAX_OFFSET_M
                or abs(yaw_error) > self.MAX_YAW_RAD):
            raise ValueError('fine_align_outside_envelope')
        return target
