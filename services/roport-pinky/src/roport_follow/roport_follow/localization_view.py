"""시험 화면용(읽기 전용): 로봇이 아는 자기 위치를 지도 위에. 로봇에 아무것도 보내지 않는다.

- 자기 위치 = TF map → base_footprint(Nav2·AMCL 이 쓰는 값) + /amcl_pose 분산.
- 라이다 /scan 점과 지역 비용 지도(/local_costmap/costmap)의 장애물 칸을 같은 map 좌표로 옮겨 함께 준다.
  지도와 라이다가 겹치면 위치가 맞는 것, 비용 지도 장애물이 로봇 앞에 있으면 Nav2 가 「collision ahead」로 멈춘다.
- 지도 그림은 yaml(지도 파일)에서 한 번 읽어 PNG 로 준다(map_png).
"""
from __future__ import annotations

import io
import math
import threading
import time
from pathlib import Path

LETHAL, INSCRIBED = 100, 99      # nav_msgs/OccupancyGrid 로 나온 비용 지도 값(254 → 100, 253 → 99)


def load_map(yaml_path):
    """지도 yaml → (meta dict, PNG bytes). 흰 = 빈 곳, 검정 = 벽(파일 그대로)."""
    import yaml
    from PIL import Image
    p = Path(yaml_path).expanduser()
    cfg = yaml.safe_load(p.read_text())
    img = Image.open(p.parent / cfg['image']).convert('L')
    buf = io.BytesIO()
    img.save(buf, format='PNG')
    meta = {'name': p.name, 'resolution': float(cfg['resolution']), 'origin': [float(v) for v in cfg['origin'][:2]],
            'width': img.width, 'height': img.height}
    return meta, buf.getvalue()


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def grid_cells(grid, tf_map_grid, center, radius_m, max_cells=2500):
    """OccupancyGrid 의 장애물(LETHAL)·내접(INSCRIBED) 칸 → map 좌표 [(x, y, v)]. center 에서 radius_m 안만."""
    info = grid.info
    w, h, res = info.width, info.height, info.resolution
    ox, oy = info.origin.position.x, info.origin.position.y
    tx, ty, tyaw = tf_map_grid
    c, s = math.cos(tyaw), math.sin(tyaw)
    lethal, inscribed = [], []
    data = grid.data
    for idx in range(w * h):
        v = data[idx]
        if v < INSCRIBED:
            continue
        gx, gy = ox + (idx % w + 0.5) * res, oy + (idx // w + 0.5) * res
        mx, my = tx + c * gx - s * gy, ty + s * gx + c * gy
        if math.hypot(mx - center[0], my - center[1]) <= radius_m:
            (lethal if v >= LETHAL else inscribed).append((round(mx, 3), round(my, 3), int(v)))
    # 장애물(lethal)을 먼저 다 담고, 남는 자리만 내접 칸(화면이 너무 무거워지지 않게)
    inscribed.sort(key=lambda q: math.hypot(q[0] - center[0], q[1] - center[1]))
    return lethal[:max_cells] + inscribed[:max(0, max_cells - len(lethal))]


class LocalizationView:
    def __init__(self, node, map_yaml, radius_m=1.5, period_s=0.25):
        import tf2_ros
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
        self.meta, self.png = load_map(map_yaml)
        self.radius, self.period = float(radius_m), float(period_s)
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, node)
        self.lock = threading.Lock()
        self.scan = self.grid = self.amcl = None
        self.amcl_at = None
        self._snap, self._snap_at = None, 0.0
        node.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, 5)
        q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        node.create_subscription(OccupancyGrid, '/local_costmap/costmap', self._on_grid, q)

    def _on_scan(self, m):
        with self.lock:
            self.scan = m

    def _on_grid(self, m):
        with self.lock:
            self.grid = m

    def _on_amcl(self, m):
        c = m.pose.covariance
        with self.lock:
            self.amcl, self.amcl_at = {'xx': c[0], 'yy': c[7], 'yaw': c[35]}, time.time()

    def _tf(self, target, source):
        import rclpy
        t = self.buf.lookup_transform(target, source, rclpy.time.Time())
        return t.transform.translation.x, t.transform.translation.y, _yaw(t.transform.rotation)

    def info(self):
        """화면용 스냅숏(최대 period_s 마다 새로 계산)."""
        now = time.time()
        if self._snap is not None and now - self._snap_at < self.period:
            return self._snap
        with self.lock:
            scan, grid, amcl, amcl_at = self.scan, self.grid, self.amcl, self.amcl_at
        out = {'map': self.meta, 'pose': None, 'scan': [], 'cells': [], 'amcl': amcl,
               'amcl_age_s': None if amcl_at is None else round(now - amcl_at, 1), 'error': None}
        try:
            rx, ry, ryaw = self._tf('map', 'base_footprint')
            out['pose'] = {'x': round(rx, 3), 'y': round(ry, 3), 'yaw_deg': round(math.degrees(ryaw), 1)}
            if scan is not None:
                lx, ly, lyaw = self._tf('map', scan.header.frame_id)
                pts = []
                for i in range(0, len(scan.ranges), 2):
                    r = scan.ranges[i]
                    if math.isfinite(r) and scan.range_min < r < min(scan.range_max, 4.0):
                        a = scan.angle_min + i * scan.angle_increment + lyaw
                        pts.append((round(lx + r * math.cos(a), 3), round(ly + r * math.sin(a), 3)))
                out['scan'] = pts
            if grid is not None:
                out['cells'] = grid_cells(grid, self._tf('map', grid.header.frame_id), (rx, ry), self.radius)
        except Exception as e:  # noqa: BLE001 — TF 가 아직 없음(시험 스택 꺼짐·처음 위치 전)
            out['error'] = f'{type(e).__name__}: {str(e)[:80]}'
        self._snap, self._snap_at = out, now
        return out
