import math
from types import SimpleNamespace as NS

import numpy as np
import pytest

from roport_smartcart.shopping_track import Track, foot_position


def test_corner_prediction_is_bounded_and_never_switches_to_a_crossing_person():
    track = Track()
    for i in range(5):
        track.update(i*.12, (.8, .04*i), True)
    assert track.target() == (.8, .16)
    track.update(.6, None, False)
    assert track.state == 'CORNER'
    assert math.dist(track.target(), (.8,.16)) == pytest.approx(.12)
    track.update(.8, (.1,-.4), False)
    assert track.state == 'AMBIGUOUS' and track.target() is None
    for i in range(3):
        track.update(.9+.12*i, (.8,.19), False)
    assert track.state == 'FOLLOWING'


def test_prediction_expires_and_stationary_occlusion_does_not_create_motion():
    track = Track()
    for i in range(5):
        track.update(i*.1, (.8, .03*i), True)
    track.update(.6,None,False)
    assert track.target() is not None
    track.update(2.5,None,False)
    assert track.target() is None
    for i in range(5):
        track.update(3+i*.1,(.8,.12),True)
    track.update(3.6,None,False)
    assert track.target() is None


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


def test_sideways_motion_can_turn_behind_a_wall_in_the_middle_of_the_image():
    track = Track()
    for i in range(4):
        track.update(.15*i, (.8,.03*i), False, center=.50+.025*i)
    track.update(.60,None,False)
    assert track.state=='CORNER' and track.target()[1] > .09
    track.update(.72,None,False,ambiguous=True)
    assert track.target() is None


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


def test_proximity_notice_waits_for_sustained_loss_or_distance_and_clears_on_confirmation():
    from roport_smartcart.shopping_track import ProximityNotice
    notice = ProximityNotice()
    assert not notice.update(0, True, 'SEARCHING', None)
    assert not notice.update(2, True, 'FOLLOWING', .41)
    assert not notice.update(3, True, 'CORNER', None)
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
        track.update(stamp, (.8, .12), False)
    assert track.state == 'FOLLOWING'
    track.update(3., (.8, .12), False)
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
    grid = NS(info=NS(width=20, height=20, resolution=.05, origin=NS(position=NS(x=-.5,y=-.5))),
              data=[0]*400)
    grid.data[10*20+11] = cost
    nav = Mock(follow_distance_m=.26, boot='boot', retry_after=0., path_failures=retry,
               pending=None, handle=None, anchor=None, last_reached=None, fault=None)
    nav.node.get_clock().now().to_msg.return_value = Time()
    yaw = math.radians(-15)
    target = (.34*math.cos(yaw), .34*math.sin(yaw))
    ShoppingNav.tick(nav, 2., True, (0.,0.,0.), target, direct_grid=grid)
    assert nav.follow_client.send_goal_async.called is direct
    assert nav.client.send_goal_async.called is not direct
    if direct:
        path = nav.follow_client.send_goal_async.call_args.args[0].path
        assert path.header.frame_id == 'map' and len(path.poses) >= 5
        for p in path.poses[1:]:
            assert math.atan2(p.pose.position.y, p.pose.position.x) == pytest.approx(yaw)
        assert math.hypot(path.poses[-1].pose.position.x, path.poses[-1].pose.position.y) == pytest.approx(.08)
