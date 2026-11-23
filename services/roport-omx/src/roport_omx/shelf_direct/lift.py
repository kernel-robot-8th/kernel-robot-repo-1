"""착지 전 손끝 들기: Panel omx_safe_start.lift_plan을 옮겼다. 계산은 panel_vendor/omx_lift.py 그대로.

정지 후 홈으로 돌아가기 전에 손끝을 지금 기울기 그대로 lift_dz(4 cm) 곧게 든다. 2층 판 아래 1층 칸 안이면
먼저 앞으로 뺀 뒤 2 cm 든다. 계산을 못 하거나 결과가 비정상(경유점 사이 관절 변화 > STEP_LIMIT, 범위 밖)이면
들지 않고 바로 홈으로 간다(빈 목록).
Mover(lifter=...)에 넘기는 callable을 만든다: (present, side) → [{관절 5개}, ...].
"""

from __future__ import annotations

import logging
import threading
from collections.abc import Callable, Mapping

from .contract import ARM_JOINTS
from .safety import in_range

log = logging.getLogger(__name__)


def panel_lifter():
    from .panel_vendor import omx_lift

    return omx_lift.Lifter(), float(getattr(omx_lift, "STEP_LIMIT", 15.0))


def make_lifter(
    dz: float = 0.04, factory: Callable = panel_lifter
) -> Callable[[Mapping[str, float], str], list[dict[str, float]]]:
    state: dict = {"tried": False, "obj": None, "limit": 15.0}
    guard = threading.Lock()

    def load():
        with guard:
            if not state["tried"]:
                state["tried"] = True
                try:
                    state["obj"], state["limit"] = factory()
                except Exception as exc:  # noqa: BLE001 - 계산기가 없으면 들지 않고 홈으로(Panel과 같음)
                    log.warning("lift planner unavailable: %s", exc)
            return state["obj"]

    def plan(present: Mapping[str, float], side: str) -> list[dict[str, float]]:
        if side not in ("left", "right") or dz <= 0:
            return []
        lifter = load()
        if lifter is None:
            return []
        own = {
            m: float(v) for m, v in present.items() if m in ARM_JOINTS or m == "gripper"
        }
        other = "right" if side == "left" else "left"
        # 계획기 FK는 양팔 관절을 받는다. 다른 팔 값은 이 팔 계획에 영향이 없어 같은 값으로 채운다(Panel과 같음)
        joints = {f"{other}_{m}": v for m, v in own.items()} | {
            f"{side}_{m}": v for m, v in own.items()
        }
        try:
            mode, path = lifter.plan(joints, side, dz)
        except Exception:  # noqa: BLE001 - plan()도 실패하면 none을 돌려준다
            return []
        if mode == "none" or not path:
            return []
        points, prev = [], {m: float(present[m]) for m in ARM_JOINTS}
        for wp in path:
            try:
                p = {m: float(wp[f"{side}_{m}"]) for m in ARM_JOINTS}
            except (KeyError, TypeError, ValueError):
                return []
            if max(abs(p[m] - prev[m]) for m in ARM_JOINTS) > state["limit"] or not all(
                in_range(v) for v in p.values()
            ):
                log.warning("%s: abnormal lift plan, going home without lifting", side)
                return []
            points.append(p)
            prev = p
        log.info("%s: lifting fingertips first (%s, %d steps)", side, mode, len(points))
        return points

    return plan
