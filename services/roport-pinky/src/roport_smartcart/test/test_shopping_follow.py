import math
from types import SimpleNamespace as NS

import numpy as np
import pytest

from roport_smartcart.shopping_track import Track, foot_position


def test_loss_revokes_motion_target_and_reacquisition_requires_three_matches():
    track = Track()
    for i in range(5):
        track.update(i*.12, (.8, .04*i))
    assert track.target() == (.8, .16)
    track.update(.6, None)
    assert track.state == 'SEARCHING' and track.target() is None
    track.update(.8, (.1,-.4))
    assert track.state == 'AMBIGUOUS' and track.target() is None
    for stamp in (.9, 1.02):
        track.update(stamp, (.8, .19))
        assert track.state == 'VERIFYING' and track.target() is None
    track.update(1.14, (.8, .19))
    assert track.state == 'FOLLOWING' and track.target() == (.8, .19)


def test_delayed_or_repeated_observation_cannot_restore_lost_target():
    track = Track()
    for i in range(5):
        track.update(i*.1, (.8, .03*i))
    track.update(.6, None)
    for stamp in (.5, .6, float('nan')):
        assert not track.update(stamp, (.8, .12))
        assert track.state == 'SEARCHING' and track.target() is None


def test_floor_intersection_uses_optical_tf_and_rejects_cut_off_feet():
    info=NS(k=[500.,0,320,0,500.,240,0,0,1.],height=480,d=[])
    camera=NS(rotation=NS(x=-.5,y=.5,z=-.5,w=.5), translation=NS(x=1.,y=-.2,z=.12))
    assert foot_position([300,170,340,340],info,camera)==pytest.approx((1.6,-.2))
    assert foot_position([300,100,340,220],info,camera) is None
    assert foot_position([300,100,340,479],info,camera) is None


def test_reference_proposals_handle_reappearance_and_two_similar_copies():
    cv2=pytest.importorskip('cv2')
    from roport_smartcart.box_detector import SelectedProposals
    original=np.zeros((320,440,3),np.uint8)
    crop=np.random.default_rng(7).integers(0,255,(100,54,3),dtype=np.uint8)
    original[80:180,50:104]=crop
    finder=SelectedProposals(original,(50/440,80/320,104/440,180/320))
    blank=np.zeros_like(original)
    assert finder.detect(blank)==[]
    blank[10:110,200:254]=crop
    blank[190:290,110:164]=crop
    assert len(finder.detect(blank))==2


def test_sideways_motion_before_occlusion_still_drops_target_immediately():
    track = Track()
    for i in range(4):
        track.update(.15*i, (.8,.03*i))
    track.update(.60, None)
    assert track.state == 'SEARCHING' and track.target() is None
    track.update(.72, None, ambiguous=True)
    assert track.state == 'AMBIGUOUS' and track.target() is None


def test_sonar_bypass_uses_only_free_known_side_between_robot_and_target():
    from roport_smartcart.shopping_track import bypass_waypoint
    grid = NS(info=NS(resolution=.02, width=100, height=100,
                      origin=NS(position=NS(x=-1.,y=-1.))),
              data=[0]*10000)
    for row in range(50):
        grid.data[row*100:(row+1)*100] = [100]*100
    assert bypass_waypoint((0,.04),(.8,.04),(.30,.04),grid)==pytest.approx((.42,.24))
    assert bypass_waypoint((0,.04),(.8,.04),(.50,.27),grid) is None
    assert bypass_waypoint((0,.04),(.45,.04),(.42,.04),grid) is None
    grid.data = [-1]*10000
    assert bypass_waypoint((0,.04),(.8,.04),(.30,.04),grid) is None


def test_seed_at_close_range_can_propose_same_figure_at_half_scale():
    cv2 = pytest.importorskip('cv2')
    from roport_smartcart.box_detector import SelectedProposals
    rng = np.random.default_rng(10)
    crop = cv2.resize(rng.integers(0, 255, (25, 16, 3), dtype=np.uint8), (112, 168))
    frame = np.zeros((420, 480, 3), dtype=np.uint8)
    frame[60:228,80:192] = crop
    finder = SelectedProposals(frame, (80/480,60/420,192/480,228/420))
    far = np.zeros_like(frame)
    far[120:204,200:256] = cv2.resize(crop,(56,84))
    assert any(abs(box['box'][0]-200)<4 and abs(box['box'][3]-204)<5
               for box in finder.detect(far))


@pytest.mark.parametrize('left', [-8, 376])
def test_selected_figure_partly_clipped_on_either_side(left):
    cv2 = pytest.importorskip('cv2')
    from roport_smartcart.box_detector import SelectedProposals
    rng = np.random.default_rng(11)
    crop = cv2.resize(rng.integers(0, 255, (25, 16, 3), dtype=np.uint8), (112, 168))
    frame = np.zeros((420, 480, 3), dtype=np.uint8)
    frame[60:228, 80:192] = crop
    finder = SelectedProposals(frame, (80/480, 60/420, 192/480, 228/420))
    edge = np.zeros_like(frame)
    a, c = max(0, left), min(480, left+112)
    edge[60:228, a:c] = crop[:, a-left:c-left]
    assert any(abs(x['box'][0]-a) <= 2 and abs(x['box'][2]-c) <= 2
               and abs(x['box'][1]-60) <= 2 for x in finder.detect(edge))


def test_proximity_notice_waits_for_sustained_loss_or_distance_and_clears_on_confirmation():
    from roport_smartcart.shopping_track import ProximityNotice
    notice = ProximityNotice()
    assert not notice.update(0, True, 'SEARCHING', None)
    assert not notice.update(2, True, 'FOLLOWING', .41)
    assert not notice.update(3, True, 'SEARCHING', None)
    assert notice.update(6.1, True, 'AMBIGUOUS', None)
    assert notice.update(6.2, True, 'FOLLOWING', .7)
    assert not notice.update(6.8, True, 'FOLLOWING', .7)
    assert not notice.update(7, True, 'FOLLOWING', .9)
    assert notice.update(10.1, True, 'FOLLOWING', .9)
    assert notice.update(11, True, 'FOLLOWING', .8)
    assert not notice.update(12, False, 'SEARCHING', None)


def test_detour_reacquisition_needs_same_map_position_and_repeated_unique_appearance():
    from roport_smartcart.session_target import SessionTracker
    frame = np.zeros((320,480,3), np.uint8)
    class Encoder:
        def encode(self, crops):
            return np.tile([1.,0.] if not self.selected else [.81, math.sqrt(1-.81**2)], (len(crops),1))
        selected = False
    encoder = Encoder()
    box = [260,115,310,215]
    tracker = SessionTracker(encoder,lambda _: [box],frame,(.1,.1,.25,.5))
    encoder.selected = True
    for _ in range(6):
        assert tracker.step(frame, near_anchor=lambda _: False)[2]['state'] == 'LOST'
    for _ in range(2):
        assert tracker.step(frame, near_anchor=lambda _: True)[2]['state'] == 'LOST'
    assert tracker.step(frame, near_anchor=lambda _: True)[2]['state'] == 'MATCH'


def test_active_costmaps_cannot_arm_an_inactive_navigation_stack():
    pytest.importorskip('nav2_msgs')
    from concurrent.futures import Future
    from roport_smartcart.shopping_nav import ShoppingNav
    nav = ShoppingNav.__new__(ShoppingNav)
    nav.maps = {'local_costmap': None, 'global_costmap': None}
    nav.checks = {}
    nav.map_verified = dict.fromkeys(nav.maps, 10.)
    nav.active_at = nav.last_check = 10.
    for active in (False, True):
        nav.active_request = Future()
        nav.active_request.set_result(NS(success=active))
        assert nav.maps_ready(11.) is active


@pytest.mark.parametrize('gap, separation, sends_goal',
                         [(.22, .249, False), (.22, .25, True), (.27, .299, False),
                          (.27, .30, True), (.34, .35, False), (.34, .37, True),
                          (.20, .249, False), (.20, .25, True)])
def test_shopping_goal_respects_configured_following_gap(gap, separation, sends_goal):
    pytest.importorskip('nav2_msgs')
    from builtin_interfaces.msg import Time
    from unittest.mock import Mock
    from roport_smartcart.shopping_nav import ShoppingNav

    nav = Mock(follow_distance_m=gap, fault=None, boot=None, anchor=None, retry_after=0., last_reached=None,
               pending=None, handle=None, guard={'owner': 'none', 'boot': 'test'})
    nav.ready.return_value = True
    nav.node.get_clock().now().to_msg.return_value = Time()
    ShoppingNav.tick(nav, 1., True, (0., 0., 0.), (separation, 0.))
    assert nav.client.send_goal_async.called == sends_goal
    if sends_goal:
        goal = nav.client.send_goal_async.call_args.args[0]
        assert goal.pose.pose.position.x == pytest.approx(separation-max(.22, gap))


def test_frozen_photo_is_examined_once_and_a_retake_discards_the_old_result():
    pytest.importorskip('nav2_msgs')
    from threading import Condition
    from unittest.mock import Mock
    from roport_smartcart.shopping_follow import Shopping

    image = np.zeros((150, 200, 3), np.uint8)
    frozen = 'first', (1., image, None, b'jpeg')
    shop = NS(lock=Condition(), frozen=frozen, detector=Mock(), node=Mock())
    shop.detector.detect.return_value = [dict(box=[90, 1, 160, 90], score=.25),
                                        dict(box=[0, 4, 45, 100], score=.8)]
    Shopping.detect_capture(shop, frozen)
    shop.detector.detect.assert_called_once_with(image)
    stamp, frame, boxes, jpeg = shop.frozen[1]
    assert frame is image and (stamp, jpeg) == (1., b'jpeg')
    assert [b['box'][0] for b in boxes] == [0., .45]
    retake = 'second', (2., image.copy(), None, b'new')
    shop.frozen = retake
    Shopping.detect_capture(shop, frozen)
    assert shop.frozen is retake


def test_onboard_inference_interval_keeps_verified_matches_but_expires_a_gap():
    track = Track()
    for stamp in (0., .22, .45, 1.33):
        track.update(stamp, (.8, .12))
    assert track.state == 'FOLLOWING'
    track.update(3., (.8, .12))
    assert track.target() is None


def test_partial_appearance_keeps_same_box_but_near_equal_candidates_wait():
    from roport_smartcart.session_target import SessionTracker
    from unittest.mock import Mock
    frame = np.zeros((200, 200, 3), np.uint8)
    encoder = Mock()
    encoder.encode.return_value = [[1., 0.]]
    boxes = [[10, 10, 70, 110]]
    tracker = SessionTracker(encoder, lambda _: boxes, frame, (.05, .05, .35, .55))
    feature = [.74, math.sqrt(1-.74**2)]
    encoder.encode.return_value = [feature]
    assert tracker.step(frame)[2]['state'] == 'MATCH'
    boxes.append([100, 10, 160, 110])
    encoder.encode.return_value = [feature, feature]
    assert tracker.step(frame)[2]['state'] == 'AMBIGUOUS'


def test_blocked_shopping_goal_waits_for_a_fresh_target_and_stops_after_two_retries(monkeypatch):
    pytest.importorskip('nav2_msgs')
    from concurrent.futures import Future
    from unittest.mock import Mock
    from action_msgs.msg import GoalStatus
    from builtin_interfaces.msg import Time
    from roport_smartcart import shopping_nav

    nav = shopping_nav.ShoppingNav.__new__(shopping_nav.ShoppingNav)
    nav.client, nav.node, nav.heartbeat = Mock(), Mock(), Mock()
    nav.node.get_clock().now().to_msg.return_value = Time()
    nav.pending = nav.handle = nav.result = nav.anchor = nav.fault = nav.last_reached = nav.boot = None
    nav.retry_after, nav.path_failures, nav.follow_distance_m = 0., 0, .26
    nav.cancelled = False
    nav.guard = {'owner': 'none', 'boot': 'test'}
    nav.ready = lambda now: True
    pose, target = (0., 0., 0.), (.55, .01)

    def result(status, code):
        nav.pending = None
        nav.result = Future()
        nav.result.set_result(NS(status=status, result=NS(error_code=code, error_msg='')))
        nav.poll()

    now = 10.
    monkeypatch.setattr(shopping_nav.time, 'monotonic', lambda: now)
    nav.tick(now, True, pose, target)
    for attempt, delay in enumerate((3., 12.), 1):
        result(GoalStatus.STATUS_ABORTED, 104)
        assert nav.fault is None and nav.path_failures == attempt
        for elapsed, enabled, point, allow in ((delay-.1, True, target, True),
                                               (delay, False, target, True),
                                               (delay, True, None, True),
                                               (delay, True, target, False)):
            nav.tick(now + elapsed, enabled, pose, point, allow_new=allow)
            assert nav.client.send_goal_async.call_count == attempt
            nav.heartbeat.assert_called_with(False, now+elapsed)
        now += delay
        nav.tick(now, True, pose, target)
    result(GoalStatus.STATUS_ABORTED, 104)
    nav.tick(now+30, True, pose, target)
    assert nav.fault == 'PATH_BLOCKED' and nav.client.send_goal_async.call_count == 3


def test_acknowledged_short_goal_is_not_resent_until_shopper_moves():
    pytest.importorskip('nav2_msgs')
    from unittest.mock import Mock
    from builtin_interfaces.msg import Time
    from roport_smartcart.shopping_nav import ShoppingNav

    nav = Mock(follow_distance_m=.26, fault=None, boot=None, anchor=None, pending=None,
               handle=None, retry_after=0., last_reached=((.05, 0.), 0.), guard={'owner':'none','boot':'test'})
    nav.node.get_clock().now().to_msg.return_value = Time()
    for stamp in (1., 1.05, 1.1):
        ShoppingNav.tick(nav, stamp, True, (0., 0., 0.), (.31, 0.))
        assert nav.anchor is None and not nav.client.send_goal_async.called
    ShoppingNav.tick(nav, 2., True, (0., 0., 0.), (.36, 0.))
    assert nav.client.send_goal_async.call_count == 1


def test_lone_barbie_at_image_edge_needs_five_consistent_observations():
    from unittest.mock import Mock
    from roport_smartcart.session_target import SessionTracker

    frame = np.zeros((480, 640, 3), np.uint8)
    box = [481, 1, 640, 466]
    encoder = Mock()
    encoder.encode.return_value = [[1., 0.]]
    tracker = SessionTracker(encoder, lambda _: [box], frame, (.40, 0., .75, .98))
    encoder.encode.return_value = [[.849, math.sqrt(1-.849**2)]]
    for _ in range(4):
        assert tracker.step(frame)[2]['state'] == 'LOST'
    assert tracker.step(frame)[2]['state'] == 'MATCH'


@pytest.mark.parametrize('old_y,new_y,detour,turn_only,should_cancel', [
    (.06, -.06, False, False, True), (-.06, .06, False, False, True),
    (.06, .065, False, False, False), (.06, -.06, True, False, False),
    (.045, .014, False, True, True),
])
def test_short_follow_goal_points_to_current_shopper(old_y, new_y, detour, turn_only, should_cancel):
    pytest.importorskip('nav2_msgs')
    from unittest.mock import Mock
    from roport_smartcart.shopping_nav import ShoppingNav

    nav = ShoppingNav.__new__(ShoppingNav)
    nav.follow_distance_m, nav.retry_after, nav.fault = .26, 0., None
    nav.origin, nav.began, nav.boot = (0., 0.), 1., 'test'
    nav.anchor, nav.destination = (.38, old_y), (.12, old_y)
    nav.detouring, nav.turn_only, nav.cancelled = detour, turn_only, False
    nav.direct_path = False
    nav.pending, nav.result = None, None
    nav.handle = Mock(accepted=True)
    nav.node, nav.client, nav.heartbeat = Mock(), Mock(), Mock()
    nav.ready = lambda _: True
    drive = nav.tick(2., True, (0., 0., 0.), (.38, new_y))
    assert nav.handle.cancel_goal_async.called is should_cancel
    assert (drive == 'HOLD') is should_cancel
    assert not nav.client.send_goal_async.called


def test_bearing_change_wraps_at_pi():
    from roport_smartcart.shopping_nav import target_bearing_change
    assert abs(target_bearing_change((0., 0., math.pi), (-.4, .01), (-.4, -.01))) < .06


@pytest.mark.parametrize('cost,retry,direct', [(0,0,True),(1,0,False),(-1,0,False),(100,0,False),(0,1,False)])
def test_clear_aisle_tracks_exact_visual_bearing_otherwise_uses_planner(cost, retry, direct):
    pytest.importorskip('nav2_msgs')
    from unittest.mock import Mock
    from builtin_interfaces.msg import Time
    from roport_smartcart.shopping_nav import ShoppingNav
    grid = make_grid()
    grid.data[10*20+11] = cost
    nav = Mock(follow_distance_m=.26, boot='boot', retry_after=0., path_failures=retry,
               pending=None, handle=None, anchor=None, last_reached=None, fault=None)
    nav.node.get_clock().now().to_msg.return_value = Time()
    nav.node.get_clock().now().nanoseconds = 10_000_000_000
    yaw = math.radians(-15)
    target = (.34*math.cos(yaw), .34*math.sin(yaw))
    ShoppingNav.tick(nav, 2., True, (0.,0.,0.), target, direct_grid=(grid, 1.8))
    assert nav.follow_client.send_goal_async.called is direct
    assert nav.client.send_goal_async.called is not direct
    if direct:
        path = nav.follow_client.send_goal_async.call_args.args[0].path
        assert path.header.frame_id == 'map' and len(path.poses) >= 5
        for p in path.poses[1:]:
            assert math.atan2(p.pose.position.y, p.pose.position.x) == pytest.approx(yaw)
        assert math.hypot(path.poses[-1].pose.position.x, path.poses[-1].pose.position.y) == pytest.approx(.08)


def make_grid(sec=9, nanosec=100_000_000):
    return NS(header=NS(frame_id='map', stamp=NS(sec=sec, nanosec=nanosec)),
              info=NS(width=20, height=20, resolution=.05,
                      origin=NS(position=NS(x=-.5, y=-.5),
                                orientation=NS(x=0., y=0., z=0., w=1.))),
              data=[0]*400)


@pytest.mark.parametrize('sec,ns,receipt,direct', [
    (9, 0, 1., True),
    (8, 999_999_999, 2., False),  # newly received, but source expired
    (10, 1, 2., False),          # future source
    (0, 0, 2., False),           # unset source
    (9, 400_000_000, .99, False),  # ROS clock not enough: receipt must also be recent
    (9, 400_000_000, 2.01, False), # future receipt
])
def test_direct_path_requires_authentic_fresh_map_and_receipt(sec, ns, receipt, direct):
    pytest.importorskip('nav2_msgs')
    from builtin_interfaces.msg import Time
    from unittest.mock import Mock
    from roport_smartcart.shopping_nav import ShoppingNav

    nav = Mock(follow_distance_m=.26, boot='boot', retry_after=0., path_failures=0,
               pending=None, handle=None, anchor=None, last_reached=None, fault=None)
    nav.node.get_clock().now().to_msg.return_value = Time()
    nav.node.get_clock().now().nanoseconds = 10_000_000_000
    ShoppingNav.tick(nav, 2., True, (0., 0., 0.), (.45, 0.),
                     direct_grid=(make_grid(sec, ns), receipt))
    assert nav.follow_client.send_goal_async.called is direct
    assert nav.client.send_goal_async.called is not direct


@pytest.mark.parametrize('fault', ['expired', 'new_obstacle'])
def test_active_direct_leg_revokes_lease_when_its_live_grid_proof_fails(fault):
    pytest.importorskip('nav2_msgs')
    from unittest.mock import Mock
    from roport_smartcart.shopping_nav import ShoppingNav

    nav = ShoppingNav.__new__(ShoppingNav)
    nav.follow_distance_m, nav.retry_after, nav.fault = .26, 0., None
    nav.origin, nav.began, nav.boot = (0., 0.), 1., 'test'
    nav.anchor, nav.destination = (.45, 0.), (.19, 0.)
    nav.detouring, nav.turn_only, nav.cancelled, nav.direct_path = False, False, False, True
    nav.pending, nav.result = None, None
    nav.handle = Mock(accepted=True)
    nav.node, nav.client, nav.heartbeat = Mock(), Mock(), Mock()
    nav.node.get_clock().now().nanoseconds = 10_000_000_000
    nav.ready = lambda _: True
    grid = make_grid(8, 0) if fault == 'expired' else make_grid()
    if fault == 'new_obstacle':
        grid.data[10*20+11] = 100
    assert nav.tick(2., True, (0., 0., 0.), (.45, 0.), direct_grid=(grid, 1.9)) == 'HOLD'
    nav.handle.cancel_goal_async.assert_called_once()
    nav.heartbeat.assert_called_once_with(False, 2.)
    nav.client.send_goal_async.assert_not_called()


def make_shop(now):
    from threading import Condition
    from unittest.mock import Mock
    from builtin_interfaces.msg import Time
    from roport_smartcart.shopping_follow import Shopping
    from roport_smartcart.shopping_nav import ShoppingNav
    from roport_smartcart.shopping_track import ProximityNotice

    shop, nav = Shopping.__new__(Shopping), ShoppingNav.__new__(ShoppingNav)
    shop.lock, shop.node, shop.nav, shop.tf = Condition(), Mock(), nav, Mock()
    shop.node.get_clock().now().nanoseconds = 10_000_000_000
    shop.tf.lookup_transform.return_value = NS(
        header=NS(stamp=Time(sec=10)),
        transform=NS(translation=NS(x=0., y=0.), rotation=NS(x=0., y=0., z=0., w=1.)))
    shop.owner, shop.touch, shop.order = 'owner-session-12345', now, 1
    shop.seed, shop.armed, shop.reason, shop.drive = object(), True, '', 'APPROACH'
    shop.packet = (now, None)
    shop.default_gap, shop.field_camera, shop.gap_selected = .26, False, True
    shop.scan_at = shop.sonar_at = shop.observed = now
    shop.track, shop.waiting, shop.generation = Track(), [], 0
    for stamp in (now-.3, now-.2, now-.1):
        shop.track.update(stamp, (.55, 0.))
    shop.observed = now-.1
    shop.proximity, shop.still_since, shop.odom_at = ProximityNotice(), None, now
    shop.frozen = shop.frame_box = shop.grid = shop.hit = shop.bypass = shop.miss_reason = None
    shop.hit_at, shop.last_display, shop.display_at = float('-inf'), '', float('-inf')
    shop.display = Mock()
    nav.follow_distance_m, nav.retry_after, nav.fault = .26, 0., None
    nav.origin, nav.began, nav.boot = (0., 0.), now-.1, 'test'
    nav.anchor, nav.destination = (.55, 0.), (.34, .15)
    nav.detouring, nav.turn_only, nav.cancelled, nav.direct_path = True, False, False, False
    nav.pending, nav.result = None, None
    nav.handle = Mock(accepted=True)
    nav.node, nav.client, nav.heartbeat = shop.node, Mock(), Mock()
    nav.ready = lambda _: True
    nav.guard_at, nav.guard = now, {'owner': 'nav2', 'boot': 'test'}
    return shop


@pytest.mark.parametrize('loss', ['searching', 'ambiguous', 'verifying', 'stale_result',
                                  'scan', 'sonar', 'camera'])
def test_unconfirmed_target_or_sensor_never_keeps_detour_lease(loss, monkeypatch):
    pytest.importorskip('nav2_msgs')
    from roport_smartcart import shopping_follow

    now = 10.
    shop = make_shop(now)
    monkeypatch.setattr(shopping_follow.time, 'monotonic', lambda: now)
    if loss in ('searching', 'ambiguous', 'verifying'):
        shop.track.update(now-.07, None, ambiguous=loss == 'ambiguous')
        if loss == 'verifying':
            shop.track.update(now-.01, (.55, 0.))
        shop.observed = now-.01
    elif loss == 'stale_result':
        shop.observed = now-.61
    elif loss == 'camera':
        shop.packet = (now-.51, None)
    else:
        setattr(shop, loss+'_at', now-.51)
    shop.tick()
    assert shop.drive == 'HOLD'
    shop.nav.handle.cancel_goal_async.assert_called_once()
    shop.nav.heartbeat.assert_called_with(False, now)
    shop.nav.client.send_goal_async.assert_not_called()


def test_late_heartbeat_disarms_then_expired_owner_cannot_be_revived(monkeypatch):
    pytest.importorskip('nav2_msgs')
    from roport_smartcart import shopping_follow

    shop = make_shop(10.)
    now = 11.499
    monkeypatch.setattr(shopping_follow.time, 'monotonic', lambda: now)
    shop.expire_operator(now)
    assert shop.armed
    now = 11.5
    key = shop.owner
    assert shop.command('heartbeat', {'session': key})['armed'] is False
    assert shop.owner == key and shop.touch == now
    shop.nav.heartbeat.assert_called_once_with(False, now)
    shop.nav.handle.cancel_goal_async.assert_called_once()
    now = 13.5
    with pytest.raises(ValueError, match='세션이 종료'):
        shop.command('heartbeat', {'session': key})
    assert shop.owner is None and shop.seed is None and not shop.armed


def test_stop_fences_a_start_waiting_for_a_confirmed_target(monkeypatch):
    pytest.importorskip('nav2_msgs')
    from concurrent.futures import ThreadPoolExecutor
    from threading import Condition, Event
    from roport_smartcart import shopping_follow

    waiting = Event()

    class WaitObserved(Condition):
        def wait(self, timeout=None):
            waiting.set()
            return super().wait(timeout)

    shop = make_shop(10.)
    shop.lock, shop.armed, shop.nav.handle = WaitObserved(), False, None
    shop.nav.guard['owner'] = 'none'
    shop.track.state = 'VERIFYING'
    monkeypatch.setattr(shopping_follow.time, 'monotonic', lambda: 10.)
    key = shop.owner
    with ThreadPoolExecutor() as workers:
        result = workers.submit(shop.command, 'start', {'session': key, 'order': 2})
        assert waiting.wait(1)
        shop.command('stop', {'session': key, 'order': 3})
        with shop.lock:
            shop.track.state = 'FOLLOWING'
            shop.lock.notify_all()
        with pytest.raises(ValueError, match='주행 준비'):
            result.result(timeout=1)
    assert shop.order == 3 and not shop.armed
    shop.nav.heartbeat.assert_called_once_with(False, 10.)


def test_replayed_costmap_does_not_refresh_receipt_or_restore_invalid_proof(monkeypatch):
    pytest.importorskip('nav2_msgs')
    from roport_smartcart import shopping_follow
    from roport_smartcart.shopping_nav import fresh_costmap

    now = 10.
    shop = make_shop(now)
    shop.grid_source_ns = 0
    monkeypatch.setattr(shopping_follow.time, 'monotonic', lambda: now)
    grid = make_grid()
    shop.costmap(grid)
    assert shop.grid == (grid, now)
    now += 1.1  # ROS clock stalls while receipt clock advances.
    shop.costmap(make_grid())
    assert shop.grid == (grid, 10.)
    assert fresh_costmap(shop.grid, 10_000_000_000, now) is None

    shop.costmap(make_grid(9, 1))
    assert shop.grid is None
    shop.costmap(grid)
    assert shop.grid is None


def test_loss_cancels_pending_goal_even_if_acceptance_arrives_in_that_tick(monkeypatch):
    pytest.importorskip('nav2_msgs')
    from concurrent.futures import Future
    from roport_smartcart import shopping_follow

    now = 10.
    shop = make_shop(now)
    monkeypatch.setattr(shopping_follow.time, 'monotonic', lambda: now)
    nav, handle = shop.nav, shop.nav.handle
    nav.handle = None
    nav.pending = Future()
    nav.pending.set_result(handle)
    handle.get_result_async.return_value = Future()
    shop.track.update(now-.01, None)
    shop.observed = now-.01
    shop.tick()
    assert shop.drive == 'HOLD' and nav.cancelled
    handle.cancel_goal_async.assert_called_once()
    nav.heartbeat.assert_called_once_with(False, now)
    nav.client.send_goal_async.assert_not_called()


@pytest.mark.parametrize('jitter_ns', [0, 40_000_000])
def test_shopping_global_cadence_keeps_active_direct_leg_until_grid_expires(jitter_ns):
    pytest.importorskip('nav2_msgs')
    from pathlib import Path
    from unittest.mock import Mock
    import yaml
    from roport_smartcart.shopping_nav import COSTMAP_MAX_AGE_NS, ShoppingNav

    overlay = Path(__file__).resolve().parents[1] / 'config/shopping_costmaps.yaml'
    params = yaml.safe_load(overlay.read_text())['global_costmap']['global_costmap'][
        'ros__parameters']
    update_ns = int(1e9 / params['update_frequency'])
    publish_ns = int(1e9 / params['publish_frequency'])
    assert COSTMAP_MAX_AGE_NS == 1_000_000_000

    # Model Jazzy's strict publication deadline on its update loop. Include
    # exact alignment (one update skipped), alternating update jitter and
    # positive DDS/consumer delay; not a benchmark of hardware or Gazebo.
    base_ns = 10_000_000_000
    sources = [base_ns]
    for index in range(1, 20):
        source = base_ns + index * update_ns + (jitter_ns if index % 2 else 0)
        if sources[-1] + publish_ns < source:
            sources.append(source)
    assert len(sources) > 4

    nav = ShoppingNav.__new__(ShoppingNav)
    nav.follow_distance_m, nav.retry_after, nav.fault = .26, 0., None
    nav.origin, nav.began, nav.boot = (0., 0.), 10., 'test'
    nav.anchor, nav.destination = (.45, 0.), (.19, 0.)
    nav.detouring, nav.turn_only, nav.cancelled, nav.direct_path = False, False, False, True
    nav.pending, nav.result = None, None
    nav.handle = Mock(accepted=True)
    nav.node, nav.client, nav.heartbeat = Mock(), Mock(), Mock()
    nav.ready = lambda _: True
    pose, target = (0., 0., 0.), (.45, 0.)
    samples = [(source, source + 80_000_000) for source in sources]

    def tick(now_ns, sample):
        source, receipt = sample
        nav.node.get_clock().now().nanoseconds = now_ns
        return nav.tick(now_ns / 1e9, True, pose, target,
                        direct_grid=(make_grid(*divmod(source, 1_000_000_000)), receipt / 1e9))

    sample_idx = 0
    for now_ns in range(samples[0][1], samples[-1][1], 20_000_000):
        while samples[sample_idx + 1][1] <= now_ns:
            sample_idx += 1
        assert tick(now_ns, samples[sample_idx]) == 'APPROACH'
    nav.handle.cancel_goal_async.assert_not_called()
    assert tick(sources[-1] + COSTMAP_MAX_AGE_NS, samples[-1]) == 'APPROACH'
    assert tick(sources[-1] + COSTMAP_MAX_AGE_NS + 1, samples[-1]) == 'HOLD'
    nav.handle.cancel_goal_async.assert_called_once()
    assert nav.heartbeat.call_args.args[0] is False
    nav.client.send_goal_async.assert_not_called()


@pytest.mark.parametrize('case, expected', [
    ('edge', 'continue'), ('center', 'hold'), ('ambiguous', 'hold'),
    ('bad_camera', 'hold'), ('blocked_point', 'hold'), ('stale', 'hold'),
    ('expired', 'hold'), ('cancelled', 'look'), ('lost_too_long', 'hold'),
    ('accepted_other_goal', 'look'), ('timeout_to_look', 'look'),
])
def test_bypass_requires_same_accepted_leg_and_turn_explaining_loss(case, expected):
    pytest.importorskip('nav2_msgs')
    shop = make_shop(10.)
    nav = shop.nav
    nav.last_reached = None
    target, via = nav.anchor, nav.destination
    shop.bypass = via, (0., 0., 0.), -16. if case == 'expired' else 8.
    info = NS(width=640,k=[0. if case == 'bad_camera' else 575.,0.,320.,0.,575.,240.,0.,0.,1.],d=[])
    yaw = 0. if case == 'center' else .55
    c, s = math.cos(yaw/2), math.sin(yaw/2)
    camera = NS(translation=NS(x=0.,y=0.,z=.1),
                rotation=NS(x=(-c-s)/2,y=(c-s)/2,z=(-c+s)/2,w=(c+s)/2))
    shop.packet, shop.floor_z = (10., None, info, camera), 0.
    stamp = {'lost_too_long': -5., 'timeout_to_look': 1.}.get(case, 9.)
    shop.track.samples.clear()
    shop.track.samples.append((stamp, target))
    shop.track.confirmed = stamp, target
    shop.track.state = 'AMBIGUOUS' if case == 'ambiguous' else 'SEARCHING'
    shop.miss_reason = 'no_detection'
    nav.cancelled = case == 'cancelled'
    if case == 'accepted_other_goal':
        nav.destination = (.3, 0.)
    grid = make_grid()
    if case == 'blocked_point':
        grid.data = [100] * len(grid.data)
    pose = (0., 0., 0. if case == 'center' else .55)
    drive_target, _, look = shop.bypass_command(10., pose, None, grid, case != 'stale')
    if expected == 'continue':
        assert drive_target == target and look is None
    elif expected == 'look':
        assert drive_target is None and look == pytest.approx((-.55, stamp))
    else:
        assert drive_target is None and look is None


def test_look_cancels_old_action_and_turns_only_once_per_sighting():
    pytest.importorskip('nav2_msgs')
    from concurrent.futures import Future
    from unittest.mock import Mock
    from action_msgs.msg import GoalStatus
    nav = make_shop(10.).nav
    nav.spin_client = Mock()
    old = nav.handle
    assert nav.look(10., 2., 7.) == 'HOLD'
    old.cancel_goal_async.assert_called_once()
    nav.spin_client.send_goal_async.assert_not_called()
    nav.handle = nav.pending = None
    future = Future()
    nav.spin_client.send_goal_async.return_value = future
    assert nav.look(10.1, 2., 7.) == 'HOLD'
    goal = nav.spin_client.send_goal_async.call_args.args[0]
    assert goal.target_yaw == pytest.approx(math.pi/3) and goal.time_allowance.sec == 5
    result = Future()
    future.set_result(Mock(accepted=True, get_result_async=lambda: result))
    assert nav.look(10.2, 1., 7.) == 'APPROACH'
    result.set_result(NS(status=GoalStatus.STATUS_SUCCEEDED))
    assert nav.look(10.3, .1, 7.) == 'HOLD'
    nav.spin_client.send_goal_async.assert_called_once()
    nav.heartbeat.assert_called_with(False, 10.3)


def test_rotation_prediction_requires_both_appearance_and_map_agreement():
    from roport_smartcart.session_target import SessionTracker
    from roport_smartcart.shopping_track import foot_pixel
    info = NS(k=[500.,0,320.,0,500.,240.,0,0,1.], height=480, width=640, d=[])
    cam = NS(rotation=NS(x=-.5,y=.5,z=-.5,w=.5), translation=NS(x=1.,y=.2,z=.12))
    assert foot_pixel((1.6,.2), info, cam, 0.) == pytest.approx((320,340))
    assert foot_pixel((.6,.2), info, cam, 0.) is None
    frame = np.zeros((320,480,3), np.uint8)
    box = [260,115,310,215]
    class Encoder:
        def encode(self, crops):
            return np.tile(self.embedding,(len(crops),1))
        embedding = [1.,0.]
    encoder = Encoder()
    tracker = SessionTracker(encoder,lambda _: [box],frame,(.1,.1,.25,.5))
    assert tracker.step(frame, near_anchor=lambda _:False, expected_box=box)[2]['state'] == 'LOST'
    assert tracker.step(frame, near_anchor=lambda _:True, expected_box=box)[2]['state'] == 'MATCH'
    encoder.embedding = [0.,1.]
    assert tracker.step(frame, near_anchor=lambda _:True, expected_box=box)[2]['state'] == 'LOST'
