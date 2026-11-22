"""Map-frame history for one explicitly selected shopper. Distances in metres."""
from collections import deque
import math


class ProximityNotice:
    """Debounce passenger guidance separately from the motion decision."""
    def __init__(self):
        self.active = False
        self.since = None

    def update(self, now, enabled, state, distance):
        if not enabled:
            self.active, self.since = False, None
            return False
        in_range = state == 'CLOSE' or (
            state == 'FOLLOWING' and distance is not None and distance <= (.75 if self.active else .85))
        needs_help = not in_range
        if needs_help == self.active:
            self.since = None
        elif self.since is None:
            self.since = now
        elif now - self.since >= (.5 if self.active else 3.):
            self.active, self.since = needs_help, None
        return self.active


class Track:
    def __init__(self):
        self.samples = deque(maxlen=12)
        self.streak = 0
        self.observation_stamp = float('-inf')
        self.state = 'SEARCHING'

    def update(self, stamp, point, ambiguous=False):
        # Loss is an observation too: an older result must not undo it.
        if not math.isfinite(stamp) or stamp <= self.observation_stamp:
            return False
        self.observation_stamp = stamp
        if self.samples:
            last, previous = self.samples[-1]
            # A full onboard YOLO + appearance pass can take nearly a second.
            if stamp - last > 1.15:
                self.streak = 0
            if point is not None and math.dist(point, previous) > min(.50, .09 + .45*(stamp-last)):
                point, ambiguous = None, True
        if point is not None and all(map(math.isfinite, point)) and not ambiguous:
            self.streak += 1
            self.samples.append((stamp, point))
            self.state = 'FOLLOWING' if self.streak >= 3 else 'VERIFYING'
        else:
            self.streak = 0
            self.state = 'AMBIGUOUS' if ambiguous else 'SEARCHING'
        return True

    def target(self):
        # History is identity evidence only; it is never an extrapolated motion target.
        return self.samples[-1][1] if self.state == 'FOLLOWING' and self.samples else None


def foot_position(box, info, camera, floor_z=0.):
    """Intersect the bottom centre ray of an undistorted image with its support plane."""
    import numpy as np
    x1, _, x2, bottom = box
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    if min(fx, fy) <= 0 or bottom >= info.height-2:
        return None
    u, v = (x1+x2)/2, bottom
    if any(info.d):
        import cv2
        if info.distortion_model not in ('plumb_bob', 'rational_polynomial'):
            return None
        u, v = cv2.undistortPoints(np.array([[[u, v]]], dtype=float),
                                   np.array(info.k).reshape(3, 3), np.array(info.d)).ravel()
    else:
        u, v = (u-cx)/fx, (v-cy)/fy
    ray = np.array([u, v, 1.])
    q, t = camera.rotation, camera.translation
    axis = np.array([q.x, q.y, q.z])
    ray += 2*np.cross(axis, np.cross(axis, ray)+q.w*ray)
    if ray[2] >= -.02 or t.z <= floor_z:
        return None
    length = (floor_z-t.z)/ray[2]
    if .10 <= length <= 1.8:
        return (float(t.x+ray[0]*length), float(t.y+ray[1]*length))
    return None


def bypass_waypoint(origin, target, hit, grid):
    """Choose a free side of a short-range echo between cart and shopper, in map metres."""
    if hit is None or grid is None:
        return None
    dx, dy = target[0]-origin[0], target[1]-origin[1]
    distance = math.hypot(dx, dy)
    if distance < .48:
        return None
    ux, uy = dx/distance, dy/distance
    hx, hy = hit[0]-origin[0], hit[1]-origin[1]
    along = hx*ux + hy*uy
    if not .10 < along < min(.55, distance-.22) or abs(hy*ux-hx*uy) > .11:
        return None

    def cost(x, y):
        meta = grid.info
        col = math.floor((x-meta.origin.position.x)/meta.resolution)
        row = math.floor((y-meta.origin.position.y)/meta.resolution)
        if 0 <= col < meta.width and 0 <= row < meta.height:
            value = grid.data[row*meta.width+col]
            if value >= 0:
                return value
        return 100

    choices = []
    for sign in (-1, 1):
        wx = hit[0] + .12*ux - sign*.20*uy
        wy = hit[1] + .12*uy + sign*.20*ux
        # Compare the swept centreline on the inflated costmap, including the exit.
        costs = []
        exit_point = hit[0]+.20*ux, hit[1]+.20*uy
        for start, end in ((origin,(wx,wy)), ((wx,wy),exit_point)):
            for i in range(1, 13):
                costs.append(cost(start[0]+i*(end[0]-start[0])/12,
                                  start[1]+i*(end[1]-start[1])/12))
        if max(costs) < 99:
            choices.append((max(costs), sum(costs), wx, wy))
    return min(choices)[2:] if choices else None
