"""Exclusive supervised Nav2 client. Only trial_guard writes motor velocity."""
import json
import math
import time
from uuid import uuid4

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath, NavigateToPose
from rclpy.action import ActionClient
from rclpy.parameter_client import AsyncParameterClient
from std_msgs.msg import String
from std_srvs.srv import Trigger


MIN_FOLLOW_GAP_M = .22
START_DELTA_M = .03


def following_distances(reference):
    gap = max(MIN_FOLLOW_GAP_M, reference)
    # A short goal still needs room for the 1.5 cm arrival tolerance.
    return gap, gap + START_DELTA_M


def target_bearing_change(pose, previous, current):
    old = math.atan2(previous[1]-pose[1], previous[0]-pose[0])
    new = math.atan2(current[1]-pose[1], current[0]-pose[0])
    return math.remainder(new-old, 2*math.pi)


RETARGET_ANGLE = math.radians(12)


COSTMAP_MAX_AGE_NS = 1_000_000_000


def costmap_stamp_ns(grid):
    stamp = grid.header.stamp
    if not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError('invalid costmap stamp')
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def fresh_costmap(snapshot, ros_now_ns, now):
    """Return authentic map/receipt evidence no more than one second old."""
    if snapshot is None:
        return None
    grid, receipt = snapshot
    try:
        meta = grid.info
        origin, q = meta.origin.position, meta.origin.orientation
        source = costmap_stamp_ns(grid)
        valid = (grid.header.frame_id == 'map' and source > 0
                 and 0 <= ros_now_ns-source <= COSTMAP_MAX_AGE_NS
                 and 0 <= now-receipt <= COSTMAP_MAX_AGE_NS/1e9
                 and math.isfinite(meta.resolution) and meta.resolution > 0
                 and meta.width > 0 and meta.height > 0
                 and len(grid.data) == meta.width * meta.height
                 and all(map(math.isfinite, (origin.x, origin.y, q.x, q.y, q.z, q.w)))
                 and q.x == q.y == q.z == 0 and abs(q.w) == 1)
    except (AttributeError, TypeError, ValueError):
        return None
    return grid if valid else None


def clear_short_leg(grid, start, end):
    """Only use a straight reference in known, inflation-free costmap cells."""
    meta = grid.info
    origin = meta.origin.position
    x0, x1 = sorted(math.floor((p[0]-origin.x)/meta.resolution) for p in (start, end))
    y0, y1 = sorted(math.floor((p[1]-origin.y)/meta.resolution) for p in (start, end))
    if x0 < 0 or y0 < 0 or x1 >= meta.width or y1 >= meta.height:
        return False
    return all(grid.data[y*meta.width+x] == 0 for y in range(y0, y1+1) for x in range(x0, x1+1))


def straight_follow_goal(start, end, heading, stamp):
    goal = FollowPath.Goal()
    goal.controller_id, goal.goal_checker_id = 'FollowPath', 'general_goal_checker'
    goal.path.header.frame_id, goal.path.header.stamp = 'map', stamp
    steps = max(1, math.ceil(math.dist(start, end)/.02))
    for index in range(steps+1):
        pose = PoseStamped()
        pose.header = goal.path.header
        pose.pose.position.x = start[0] + (end[0]-start[0])*index/steps
        pose.pose.position.y = start[1] + (end[1]-start[1])*index/steps
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(heading/2), math.cos(heading/2)
        goal.path.poses.append(pose)
    return goal


class ShoppingNav:
    def __init__(self, node, follow_distance_m=.34):
        self.node = node
        self.follow_distance_m = follow_distance_m
        self.client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        self.follow_client = ActionClient(node, FollowPath, 'follow_path')
        self.lease = node.create_publisher(String, 'trial/lease', 1)
        node.create_subscription(String, 'trial/guard_state', self.guard_state, 1)
        self.guard = {}
        self.guard_at = self.last_graph = float('-inf')
        self.graph_ok = False
        self.boot, self.session, self.sequence = None, uuid4().hex, 0
        self.pending = self.handle = self.result = None
        self.cancelled = False
        self.destination = self.anchor = None
        self.detouring = self.turn_only = self.direct_path = False
        self.fault = None
        self.path_failures = 0
        self.retry_after = 0.
        self.last_reached = None
        self.maps = {name: AsyncParameterClient(node, f'{name}/{name}')
                     for name in ('local_costmap','global_costmap')}
        self.checks = {}
        self.map_verified = {}
        self.last_check = float('-inf')
        self.manager = node.create_client(Trigger, 'trial_lifecycle_manager/is_active')
        self.active_request = None
        self.active_at = float('-inf')

    def guard_state(self, msg):
        try:
            body = json.loads(msg.data)
            if not isinstance(body['boot'], str) or not math.isfinite(body['sent_at']):
                return
            if body['boot'] == self.guard.get('boot') and body['sent_at'] <= self.guard['sent_at']:
                return
            self.guard, self.guard_at = body, time.monotonic()
        except (ValueError, KeyError, TypeError):
            return

    def maps_ready(self, now):
        if self.active_request is not None and self.active_request.done():
            try:
                active = self.active_request.result().success
            except (AttributeError, RuntimeError):
                active = False
            self.active_at = now if active else float('-inf')
            self.active_request = None
        for name, future in list(self.checks.items()):
            if future.done():
                try:
                    plugins, enabled, keep, sonar, topics = future.result().values
                    valid = ('sonar_layer' in plugins.string_array_value and enabled.bool_value
                             and keep.double_value == 12. and sonar.string_value == 'roport_range_layer::FreshRangeLayer'
                             and '/us_sensor/range' in topics.string_array_value)
                except (AttributeError, TypeError, ValueError, RuntimeError):
                    valid = False
                self.map_verified[name] = now if valid else float('-inf')
                del self.checks[name]
        if now-self.last_check > 2:
            self.last_check = now
            if self.active_request is not None:
                self.active_request.cancel()
                self.active_request = None
            if self.manager.service_is_ready():
                self.active_request = self.manager.call_async(Trigger.Request())
            else:
                self.active_at = float('-inf')
            for name, client in self.maps.items():
                if name not in self.checks and client.services_are_ready():
                    self.checks[name] = client.get_parameters(['plugins','sonar_layer.enabled',
                        'sonar_layer.observation_keep_time','sonar_layer.plugin','sonar_layer.topics'])
        return now-self.active_at < 5 and all(now-self.map_verified.get(name,float('-inf')) < 5 for name in self.maps)

    def ready(self, now):
        if now-self.last_graph > .25:
            def sole(topic, owner):
                pubs = self.node.get_publishers_info_by_topic(topic)
                return len(pubs) == 1 and pubs[0].node_name == owner
            self.graph_ok = sole('cmd_vel', 'trial_guard') and sole('trial/lease', self.node.get_name())
            self.last_graph = now
        maps_ready = self.maps_ready(now)
        return (maps_ready and self.graph_ok and now-self.guard_at < .3 and self.guard.get('blocked') is False
                and self.guard.get('owner') in ('none', 'nav2') and self.client.server_is_ready()
                and (self.boot is None or self.boot == self.guard.get('boot')))

    def cancel(self):
        if self.pending or self.handle:
            if not self.cancelled and self.handle:
                self.handle.cancel_goal_async()
            self.cancelled = True

    def poll(self):
        if self.pending and self.pending.done():
            self.handle, self.pending = self.pending.result(), None
            if self.handle.accepted:
                self.result = self.handle.get_result_async()
                if self.cancelled:
                    self.handle.cancel_goal_async()
            else:
                self.handle = self.anchor = None
                self.fault = 'NAV_REJECTED'
        if self.result and self.result.done():
            response = self.result.result()
            if not self.cancelled and response.status == GoalStatus.STATUS_SUCCEEDED:
                self.last_reached = (self.destination, self.goal_yaw)
                if not self.turn_only:
                    self.path_failures = 0
            elif not self.cancelled:
                outcome = response.result
                blocked = outcome.error_code in (104, 105, 106, 205, 206, 208)
                if blocked and self.path_failures < 2:
                    delay = (3., 12.)[self.path_failures]
                    self.path_failures += 1
                    self.retry_after = time.monotonic() + delay
                    self.node.get_logger().warning(
                        f'PATH_BLOCKED code={outcome.error_code} retry={self.path_failures}/2 '
                        f'wait={delay}s goal={self.destination} {outcome.error_msg}')
                else:
                    self.fault = 'PATH_BLOCKED' if blocked else 'NAV_FAILED'
                    self.node.get_logger().error(
                        f'{self.fault} error_code={outcome.error_code} goal={self.destination} {outcome.error_msg}')
            self.result = self.handle = self.anchor = None

    def heartbeat(self, moving, now):
        if self.boot and self.graph_ok and self.boot == self.guard.get('boot') and now-self.guard_at < .3:
            self.sequence += 1
            data = dict(boot=self.boot, owner='nav2' if moving else 'none', session=self.session,
                        seq=self.sequence, sent_at=self.guard['sent_at']+now-self.guard_at)
            self.lease.publish(String(data=json.dumps(data)))

    def tick(self, now, enabled, pose, target, via=None, allow_new=True, direct_grid=None):
        # Latch cancellation before consuming a just-arrived goal acceptance.
        if not enabled or pose is None or target is None:
            self.cancel()
        self.poll()
        ready = self.ready(now)
        if not enabled or not ready or pose is None or target is None or self.fault or now < self.retry_after:
            self.cancel()
            self.heartbeat(False, now)
            return 'HOLD'
        if self.boot is None:
            if self.guard['owner'] != 'none':
                return 'HOLD'
            self.boot = self.guard['boot']
        dx, dy = target[0]-pose[0], target[1]-pose[1]
        separation = math.hypot(dx, dy)
        gap, advance_at = following_distances(self.follow_distance_m)
        bearing = math.remainder(math.atan2(dy,dx)-pose[2], 2*math.pi)
        grid = (fresh_costmap(direct_grid, self.node.get_clock().now().nanoseconds, now)
                if direct_grid is not None else None)
        if (self.anchor and self.direct_path and not self.cancelled
                and (grid is None or not clear_short_leg(grid, pose[:2], self.destination))):
            self.node.get_logger().info(
                'SHOP_HOLD reason=costmap_stale' if grid is None else 'SHOP_HOLD reason=costmap_blocked')
            self.cancel()
        if self.anchor and (now-self.began > 60 or math.dist(pose[:2], self.origin) > .95):
            self.fault = 'NAV_LIMIT'
            self.cancel()
        elif self.anchor and not self.cancelled:
            change = target_bearing_change(pose, self.anchor, target)
            if not self.detouring and (abs(change) > RETARGET_ANGLE or math.dist(target, self.anchor) > .18):
                self.node.get_logger().info(f'SHOP_RETARGET delta={math.degrees(change):.0f}deg '
                                            f'target_bearing={math.degrees(bearing):.0f}deg')
                self.cancel()
            elif self.turn_only and not self.detouring and abs(bearing) < math.radians(8):
                self.cancel()
            elif not self.turn_only and (separation <= gap + .015 or
                    not self.detouring and math.dist(pose[:2], self.destination) < .015):
                self.cancel()
        if allow_new and not self.pending and not self.handle and not self.fault:
            self.cancelled = False
            travel = min(.8, separation-gap)
            advance = separation >= advance_at - 1e-9
            self.turn_only = not advance and abs(bearing) > .32
            if advance or self.turn_only:
                if self.turn_only:
                    travel = 0.
                yaw = math.atan2(dy, dx)
                self.destination = (pose[0]+travel*math.cos(yaw), pose[1]+travel*math.sin(yaw))
                self.detouring = via is not None
                if via is not None:
                    self.destination = via
                    yaw = math.atan2(target[1]-via[1], target[0]-via[0])
                    self.node.get_logger().info(f'SONAR_BYPASS via={via} target={target}')
                direct = False
                if (advance and grid is not None and via is None and not self.path_failures
                        and self.follow_client.server_is_ready()):
                    step = min(.25, travel)
                    end = (pose[0]+step*math.cos(yaw), pose[1]+step*math.sin(yaw))
                    if clear_short_leg(grid, pose[:2], end):
                        self.destination, direct = end, True
                # Grid discretisation can acknowledge a short leg before the base moves.
                if self.last_reached:
                    position, heading = self.last_reached
                    if (math.dist(position, self.destination) < .015 and math.dist(position, pose[:2]) < .06
                            and abs(math.remainder(yaw-heading, 2*math.pi)) < .08):
                        self.heartbeat(False, now)
                        return 'HOLD'
                self.last_reached = None
                self.anchor, self.origin, self.began = target, pose[:2], now
                self.goal_yaw = yaw
                self.retry_after = 0.
                stamp = self.node.get_clock().now().to_msg()
                self.direct_path = direct
                if direct:
                    goal = straight_follow_goal(pose[:2], self.destination, yaw, stamp)
                    self.pending = self.follow_client.send_goal_async(goal)
                    mode = 'follow_path'
                else:
                    goal = NavigateToPose.Goal()
                    goal.pose.header.frame_id, goal.pose.header.stamp = 'map', stamp
                    goal.pose.pose.position.x, goal.pose.pose.position.y = self.destination
                    goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
                    self.pending = self.client.send_goal_async(goal)
                    mode = 'navigate_to_pose'
                self.node.get_logger().info(
                    f'SHOP_GOAL mode={mode} bearing={math.degrees(bearing):.0f}deg separation={separation:.2f}m '
                    f'turn_only={self.turn_only} detour={self.detouring} goal={self.destination}')
        active = bool(self.handle and self.handle.accepted and not self.cancelled)
        self.heartbeat(active, now)
        return 'APPROACH' if active else 'HOLD'
