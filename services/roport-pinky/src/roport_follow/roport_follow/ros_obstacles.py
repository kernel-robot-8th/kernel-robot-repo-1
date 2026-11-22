"""인형 바닥 점을 ROS 2 PointCloud2 로 보낸다(노트북에서 실행, 로봇 Nav2 비용 지도가 받음).

모터 명령·Nav2 목표는 보내지 않는다. 점만 보내고, 피해 가는 경로는 로봇의 Nav2 가 정한다.
시각: 프레임 머리의 로봇 시각(X-Robot-Time, 로봇 벽시계 = 로봇 ROS 시각)을 그대로 쓴다. 노트북 시계가 어긋나도
비용 지도가 그 시각의 TF 로 점을 옮긴다. 로봇 시각이 없으면(동영상 재생 시험) 이 노드의 현재 시각을 쓴다.
밀린 영상은 보내지 않는다(안 보내면 로봇 비용 지도가 expected_update_rate 뒤 「최신 아님」이 됨 — 그때 Nav2 가 멈추는지는
실로봇에서 확인할 것).
- 노트북 안 지연: 받은 뒤 max_age_s 넘게 지난 영상.
- 로봇 → 노트북 지연: recv_time − robot_time(시계 차 + 전송 지연)의 지금까지 최솟값보다 max_age_s 넘게 크면 밀린 것.
  비용 지도는 스탬프가 persistence(1 s)보다 오래된 점을 조용히 버리면서도 「최신」으로 두므로 여기서 거른다.
- 로봇 영상인데 로봇 시각이 없으면(require_robot_time) 보내지 않는다(노트북 시계로 찍으면 시계가 틀린 기체에서 TF 가 버림).
"""
from __future__ import annotations

import collections
import math
import time

from .obstacles import ObstacleConfig, clear_fov_deg, clear_points, clear_ring, doll_obstacles

DEFAULT_TOPIC = '/doll_obstacles'
DEFAULT_FRAME = 'base_footprint'


class ObstacleSink:
    """세션에서 새 프레임마다 handle() 을 부른다. publish(points, stamp_s_or_None) 는 바꿔 끼울 수 있음(시험용)."""

    def __init__(self, cam, cfg: ObstacleConfig, publish, max_age_s=0.5, require_robot_time=False,
                 motion=None, pose_lookup=None, memory=None):
        cfg.validate()
        # 인형 기억(doll_memory.DollMemory, 11-21): 본 인형을 노트북이 기억해 매번 다시 보내고 둘레를 모두 지운다.
        # 로봇 Nav2 인형 층은 min range 0·persistence 0 이어야 한다(nav2_doll.layer_params 기본값).
        self.memory = memory
        self._ring = clear_ring(cam, cfg) if memory is not None else []
        # 움직이는 인형 예측(FFT1-190, moving.MovingTracker). pose_lookup(로봇 시각) → odom 자세 또는 None
        self.motion, self.pose_lookup = motion, pose_lookup
        # 고객(추종 대상) 둘레에서 장애물로 치지 않는 반경. 11-20 밤: 0.25 면 고객 바로 앞 어린이 인형도 빠져 못 피함 → 0.15.
        # 11-21: 고객 인형(adult)의 YOLO 바닥 투영이 카드 위치와 12~20 cm 달라 0.15 로는 고객 자신이 장애물로 찍힘
        # → 종류별: 고객과 같은 종류(adult)는 0.30, 그 밖(child)은 0.15
        self.exclude_radius_m, self.excluded = 0.15, 0
        self.exclude_radius_by_class = {'adult': 0.30}
        self.moving_info, self.predicted = [], 0
        self.cam, self.cfg, self.publish, self.max_age_s = cam, cfg, publish, float(max_age_s)
        self.require_robot_time = bool(require_robot_time)
        self.sent = self.skipped = 0
        self.lag_s = None       # 이번 영상의 로봇 → 노트북 지연(최솟값 대비, 화면 표시용)
        self._min_offset = None
        self.last = []          # 마지막으로 보낸 인형 목록(화면 표시용)
        # 시험 화면: 일부러 멈춤(끊김 시험), 보낸 시각(초당 횟수·마지막 뒤 경과), 설정 표시(tool 이 채움)
        self.paused = False
        self.last_sent_at = None
        self._sent_times = collections.deque(maxlen=50)
        self.meta = {}

    def exclude_radius(self, name):
        """고객 둘레에서 이 종류의 인형을 장애물에서 빼는 반경(m)."""
        return self.exclude_radius_by_class.get(name, self.exclude_radius_m)

    def info(self, now=None):
        """시험 화면용 상태(cm 단위). 점은 3개 중 1개만."""
        now = time.time() if now is None else now
        recent = [t for t in self._sent_times if now - t <= 2.0]
        cfg = self.cfg
        left, right = clear_fov_deg(self.cam, cfg)
        return {
            'sent': self.sent, 'stale': self.skipped, 'paused': self.paused, 'excluded_customer': self.excluded,
            'lag_ms': None if self.lag_s is None else round(self.lag_s * 1000),
            'rate_hz': round(len(recent) / 2.0, 1),
            'last_age_s': None if self.last_sent_at is None else round(now - self.last_sent_at, 2),
            'dolls': [{'name': o.name, 'conf': round(o.conf, 2), 'x_cm': round(o.x_m * 100, 1),
                       'y_cm': round(o.y_m * 100, 1), 'width_cm': round(o.width_m * 100, 1), 'feet_cut': o.cut}
                      for o in self.last],
            'points_cm': [[round(x * 100, 1), round(y * 100, 1)] for o in self.last for x, y, _ in o.points[::3]],
            'geom': {'cam_x_cm': round(cfg.cam_x_m * 100, 1), 'max_range_cm': round(cfg.max_range_m * 100),
                     'clear_range_cm': round(cfg.clear_range_m * 100), 'fov_left_deg': round(left, 1),
                     'fov_right_deg': round(right, 1), 'min_conf': cfg.min_conf},
            'meta': dict(self.meta),
            'memory': None if self.memory is None else self.memory.info(),
            'moving': None if self.motion is None else {
                'pose_ok': getattr(self, 'moving_pose_ok', False), 'predicted_points': self.predicted,
                'tracks': self.moving_info, 'horizon_s': self.motion.cfg.horizon_s},
        }

    def handle(self, frame, dets, exclude=None):
        """exclude = 추종 중인 고객 위치 (x, y) base_footprint. 그 exclude_radius_m 안 인형은 장애물에서 뺀다
        (고객 카드를 단 인형 = 따라갈 대상. 장애물로 칠하면 Nav2 목표가 장애물 위에 놓임)."""
        if self.paused:                 # 끊김 시험: 보내지 않음(로봇 비용 지도가 「최신 아님」이 되는지 봄)
            return None
        has_robot_time = math.isfinite(frame.robot_time)
        if time.time() - frame.recv_time > self.max_age_s or (self.require_robot_time and not has_robot_time):
            self.skipped += 1
            return None
        if has_robot_time:
            offset = frame.recv_time - frame.robot_time
            self._min_offset = offset if self._min_offset is None else min(self._min_offset, offset)
            self.lag_s = offset - self._min_offset
            if self.lag_s > self.max_age_s:
                self.skipped += 1
                return None
        obs = doll_obstacles(self.cam, dets, self.cfg)
        self.excluded = 0
        if exclude is not None:
            keep = [o for o in obs
                    if math.hypot(o.x_m - exclude[0], o.y_m - exclude[1]) > self.exclude_radius(o.name)]
            self.excluded = len(obs) - len(keep)
            obs = keep
        stamp = frame.robot_time if has_robot_time else None
        if self.memory is not None:
            pose = self.pose_lookup(stamp) if (self.pose_lookup is not None and stamp is not None) else None
            kept = self.memory.update(stamp if stamp is not None else frame.recv_time, pose, obs, exclude,
                                      self.exclude_radius)
            pts = [q for o in kept for q in o['points']] + clear_points(self.cam, self.cfg) + self._ring
        else:
            pts = [q for o in obs for q in o.points] + clear_points(self.cam, self.cfg)
        if self.motion is not None:
            pose = self.pose_lookup(stamp) if (self.pose_lookup is not None and stamp is not None) else None
            t = stamp if stamp is not None else frame.recv_time
            tracks = self.motion.update(t, obs, pose)
            extra = self.motion.predicted_points(pose, tracks)
            pts = pts + extra
            self.predicted = len(extra)
            self.moving_info = self.motion.summary(pose, tracks) if pose is not None else []
            self.moving_pose_ok = pose is not None
        self.publish(pts, stamp)
        self.sent += 1
        self.last_sent_at = time.time()
        self._sent_times.append(self.last_sent_at)
        self.last = obs
        return obs


def make_ros_node(node_name):
    """rclpy 노드 하나. ROS 환경(source /opt/ros/jazzy/setup.bash)이 필요하다."""
    try:
        import rclpy
        from rclpy.signals import SignalHandlerOptions
    except ImportError as e:
        raise RuntimeError('rclpy 없음: 먼저 source /opt/ros/jazzy/setup.bash 하고 같은 터미널에서 실행') from e
    if not rclpy.ok():
        # rclpy 기본 신호 처리는 Ctrl+C·SIGTERM 을 가로채 ROS 만 끄고 웹 도구는 계속 돌게 만든다 → 끔
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    return rclpy.create_node(node_name)


def ros_stamp(node, stamp_s):
    """로봇 시각(s) → builtin_interfaces/Time. None 이면 이 노드의 지금 시각."""
    from builtin_interfaces.msg import Time
    if stamp_s is None:
        return node.get_clock().now().to_msg()
    sec = int(math.floor(stamp_s))
    return Time(sec=sec, nanosec=min(int(round((stamp_s - sec) * 1e9)), 999_999_999))


class RosCloudPublisher:
    """PointCloud2 발행자. node 를 주면 그 노드를 함께 쓰고(닫지 않음), 없으면 새로 만든다."""

    def __init__(self, topic=DEFAULT_TOPIC, frame_id=DEFAULT_FRAME, node_name='doll_obstacles', node=None):
        self._own = node is None
        self.node = make_ros_node(node_name) if node is None else node
        from sensor_msgs.msg import PointCloud2
        self.pub = self.node.create_publisher(PointCloud2, topic, 5)
        self.topic, self.frame_id = topic, frame_id

    def __call__(self, points, stamp_s=None):
        from sensor_msgs_py import point_cloud2
        from std_msgs.msg import Header
        msg = point_cloud2.create_cloud_xyz32(Header(stamp=ros_stamp(self.node, stamp_s), frame_id=self.frame_id),
                                              points)
        self.pub.publish(msg)

    def close(self):
        if self._own:
            close_ros_node(self.node)


def close_ros_node(node):
    import rclpy
    try:
        node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


class OdomBuffer:
    """노트북에서 /odom 을 받아 최근 3 s 자세를 쌓고, 로봇 시각의 자세(x, y, yaw)를 보간해 준다(읽기만).
    시각이 범위 밖(오래됐거나 0.2 s 넘게 앞섬)이면 None → 예측하지 않음."""

    def __init__(self, node=None, topic='/odom', keep_s=3.0, lead_s=0.2):
        self.buf = collections.deque()
        self.keep_s, self.lead_s = float(keep_s), float(lead_s)
        if node is not None:
            from nav_msgs.msg import Odometry
            from rclpy.qos import qos_profile_sensor_data
            node.create_subscription(Odometry, topic, self._on_odom, qos_profile_sensor_data)

    def _on_odom(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.add(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.pose.position.x, m.pose.pose.position.y, yaw)

    def add(self, t, x, y, yaw):
        if self.buf and t <= self.buf[-1][0]:
            return
        self.buf.append((t, x, y, yaw))
        while self.buf and self.buf[-1][0] - self.buf[0][0] > self.keep_s:
            self.buf.popleft()

    def __call__(self, t):
        b = list(self.buf)
        if not b or t < b[0][0] or t > b[-1][0] + self.lead_s:
            return None
        if t >= b[-1][0]:
            return b[-1][1:]
        for (t0, x0, y0, a0), (t1, x1, y1, a1) in zip(b, b[1:]):
            if t0 <= t <= t1:
                k = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
                yaw = math.atan2((1 - k) * math.sin(a0) + k * math.sin(a1), (1 - k) * math.cos(a0) + k * math.cos(a1))
                return x0 + k * (x1 - x0), y0 + k * (y1 - y0), yaw
        return None


class RosSpinner:
    """구독(odom)을 받으려고 노드를 뒤쪽 스레드에서 돌린다. 발행만 할 때는 필요 없다."""

    def __init__(self, node):
        import threading
        from rclpy.executors import SingleThreadedExecutor
        self.ex = SingleThreadedExecutor()
        self.ex.add_node(node)
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        try:
            self.ex.spin()
        except Exception:  # noqa: BLE001 — 종료 중 rclpy shutdown
            pass

    def close(self):
        self.ex.shutdown(timeout_sec=1.0)
