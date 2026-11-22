"""추종 대상(고객 카드 바로 아래 바닥 점)을 ROS 2 로 보낸다(노트북에서 실행). FFT1-188.

- /follow/target  geometry_msgs/PointStamped, frame base_footprint, 시각 = 영상의 로봇 시각. 카드를 이번 영상에서
  받아들였을 때만 보낸다(못 보면 안 보냄 → 로봇 쪽은 마지막 목표를 유지하다가 status 로 놓침을 안다).
- /follow/status  std_msgs/String(JSON) 영상마다: state(searching|tracking|lost), card_id, age_s, 위치(cm) 등.
  로봇 쪽 추종 실행부는 lost 나 status 끊김(노트북 꺼짐)이면 Nav2 추종을 멈추고 놓침을 보고한다.
- 모터 명령·Nav2 목표는 보내지 않는다. 밀린 영상(받은 뒤 max_age_s 넘음)은 계산·발행하지 않는다.
"""
from __future__ import annotations

import collections
import json
import math
import time

from .follow import FollowConfig, FollowTracker

DEFAULT_TARGET_TOPIC = '/follow/target'
DEFAULT_STATUS_TOPIC = '/follow/status'
DEFAULT_FRAME = 'base_footprint'


class FollowSink:
    """세션이 새 영상마다 handle(frame, cards) 를 부른다. publish(status_dict, stamp_or_None) 는 바꿔 끼울 수 있음."""

    def __init__(self, cam, cfg: FollowConfig, publish, max_age_s=0.5, require_robot_time=False, pose_lookup=None):
        self.pose_lookup = pose_lookup      # 로봇 시각 → odom 자세(인형으로 잇기에서 로봇 움직임을 뺌) 또는 None
        self.tracker = FollowTracker(cam, cfg)
        self.cfg, self.publish, self.max_age_s = cfg, publish, float(max_age_s)
        self.require_robot_time = bool(require_robot_time)
        self.sent = self.skipped = 0
        self.last = None
        self._sent_times = collections.deque(maxlen=50)
        self.meta = {}

    def handle(self, frame, cards, dolls=None):
        has_robot_time = math.isfinite(frame.robot_time)
        if time.time() - frame.recv_time > self.max_age_s or (self.require_robot_time and not has_robot_time):
            self.skipped += 1
            return None
        t = frame.robot_time if has_robot_time else frame.recv_time
        pose = self.pose_lookup(t) if (self.pose_lookup is not None and has_robot_time) else None
        st = self.tracker.update(t, cards, dolls=dolls, pose=pose)
        self.publish(st, t if has_robot_time else None)
        self.sent += 1
        self._sent_times.append(time.time())
        self.last = st
        return st

    def exclude_point(self):
        """따라가는 중인 고객 위치(인형 장애물에서 뺄 곳). 이번 영상에서 고객을 확인했을 때만 준다.

        놓쳤거나 아직 못 찾았거나, 가려져서 이번 영상에 안 보이면(tracking, seen=false) None: 예전 차체 기준 좌표는
        로봇이 움직이면 지금 고객 자리와 다르고, 그 자리에 새로 온 다른 인형을 장애물에서 빼 버린다."""
        st = self.last
        if not st or st.get('state') != 'tracking' or not st.get('seen') or 'x_m' not in st:
            return None
        return st['x_m'], st['y_m']

    def info(self, now=None):
        now = time.time() if now is None else now
        recent = [x for x in self._sent_times if now - x <= 2.0]
        out = dict(self.last or {'state': 'searching', 'card_id': self.cfg.card_id})
        out.update(sent=self.sent, stale=self.skipped, rate_hz=round(len(recent) / 2.0, 1), meta=dict(self.meta))
        return out


class RosFollowPublisher:
    def __init__(self, node, target_topic=DEFAULT_TARGET_TOPIC, status_topic=DEFAULT_STATUS_TOPIC,
                 frame_id=DEFAULT_FRAME):
        from geometry_msgs.msg import PointStamped
        from std_msgs.msg import String
        self.node, self.frame_id = node, frame_id
        self.target = node.create_publisher(PointStamped, target_topic, 5)
        self.status = node.create_publisher(String, status_topic, 5)
        self.target_topic, self.status_topic = target_topic, status_topic

    def __call__(self, st, stamp_s=None):
        from geometry_msgs.msg import PointStamped
        from std_msgs.msg import String
        from .ros_obstacles import ros_stamp
        stamp = ros_stamp(self.node, stamp_s)
        if st.get('seen') and 'x_m' in st:
            msg = PointStamped()
            msg.header.stamp, msg.header.frame_id = stamp, self.frame_id
            msg.point.x, msg.point.y = float(st['x_m']), float(st['y_m'])
            self.target.publish(msg)
        body = dict(st, stamp=stamp.sec + stamp.nanosec * 1e-9, frame_id=self.frame_id)
        self.status.publish(String(data=json.dumps(body, ensure_ascii=False)))
