"""Nav2 profile generation is independent of ROS and the robot."""

import copy
import json
import math
from pathlib import Path

import pytest
import yaml

from roport_smartcart.profiles import build_params


ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT.parent / 'pinky_pro/pinky_navigation/params/nav2_params.yaml'
OVERRIDES = ROOT / 'config/nav2_overrides.yaml'
FOOTPRINT = [[.09, .08], [.09, -.08], [-.1, -.08], [-.1, .08]]


def profile(mode='sim', **kwargs):
    return build_params(BASE, OVERRIDES, mode, **kwargs)


def clocks(tree):
    for key, value in tree.items():
        if key == 'ros__parameters':
            yield value['use_sim_time']
        elif isinstance(value, dict):
            yield from clocks(value)


def test_sim_merge_clocks_pose_and_upstream_unchanged():
    original = BASE.read_bytes()
    p = profile()
    assert BASE.read_bytes() == original
    assert all(clocks(p))
    a = p['amcl']['ros__parameters']
    assert a['set_initial_pose'] is True
    assert a['initial_pose'] == dict(x=.3, y=-.3, z=0., yaw=0.)
    assert (a['update_min_a'], a['update_min_d']) == (.01, .05)
    for name, scaling in [('local_costmap', 5.), ('global_costmap', 10.)]:
        c = p[name][name]['ros__parameters']
        assert c['inflation_layer']['inflation_radius'] == 1.
        assert c['inflation_layer']['cost_scaling_factor'] == scaling
        assert json.loads(c['footprint']) == [[.08, .08], [.08, -.08],
                                             [-.08, -.08], [-.08, .08]]
        assert c['inflation_layer']['plugin'] == 'nav2_costmap_2d::InflationLayer'
    assert p['local_costmap']['local_costmap']['ros__parameters']['resolution'] == .01
    f = p['controller_server']['ros__parameters']['FollowPath']
    assert f['desired_linear_vel'] == .08
    assert f['inflation_cost_scaling_factor'] == 5.
    assert f['use_collision_detection'] is True
    assert p['planner_server']['ros__parameters']['GridBased']['allow_unknown'] is False
    p['amcl']['ros__parameters']['alpha1'] = 999
    assert profile()['amcl']['ros__parameters']['alpha1'] != 999


def test_real_requires_explicit_geometry_and_each_speed():
    for omit in ('footprint', 'max_linear', 'max_angular'):
        args = dict(footprint=FOOTPRINT, max_linear=.03, max_angular=.2)
        args.pop(omit)
        with pytest.raises(ValueError, match='explicit'):
            profile('real', **args)


def test_real_no_automatic_initialization_and_consistent_caps():
    footprint = copy.deepcopy(FOOTPRINT)
    p = profile('real', footprint=footprint, max_linear=.02, max_angular=.1)
    assert footprint == FOOTPRINT
    assert not any(clocks(p))
    a = p['amcl']['ros__parameters']
    assert a['set_initial_pose'] is False
    assert 'initial_pose' not in a
    f = p['controller_server']['ros__parameters']['FollowPath']
    assert f['desired_linear_vel'] == .02
    assert f['rotate_to_heading_angular_vel'] == .1
    assert f['min_approach_linear_velocity'] <= .02
    assert f['regulated_linear_scaling_min_speed'] <= .02
    b = p['behavior_server']['ros__parameters']
    assert 0 < b['min_rotational_vel'] <= b['max_rotational_vel'] == .1
    s = p['velocity_smoother']['ros__parameters']
    assert s['max_velocity'] == [.02, 0., .1]
    assert s['min_velocity'] == [-.02, 0., -.1]
    assert json.loads(p['local_costmap']['local_costmap']['ros__parameters'][
        'footprint']) == footprint


@pytest.mark.parametrize('mode', ['', 'REAL', 'gazebo', None])
def test_unknown_mode(mode):
    with pytest.raises(ValueError, match='mode'):
        profile(mode)


@pytest.mark.parametrize('name', ['max_linear', 'max_angular'])
@pytest.mark.parametrize('value', [0, -1., math.nan, math.inf, True, '0.1'])
def test_invalid_speed(name, value):
    with pytest.raises(ValueError, match='speed'):
        profile(**{name: value})


@pytest.mark.parametrize('points', [
    [], [[0, 0], [1, 1]], [[0, 0], [1, 1], [2, 2]],
    [[0, 0], [1, 0], [1, 0]],
    [[0, 0], [1, 1], [0, 1], [1, 0]],
    [[0, 0], [1, 0], [.2, .2], [0, 1]],
    [[0, 0], [1, 0], [math.nan, 1]],
    [[0, 0], [1, 0], [0, math.inf]],
    [[0, 0], [True, 0], [0, 1]],
    [[0, 0], [1, 0], ['0', 1]],
    [[0, 0], [1, 0], [0, 1, 2]],
    '[[0,0],[1,0],[0,1]]',
])
def test_invalid_footprint(points):
    with pytest.raises(ValueError, match='footprint'):
        profile(footprint=points)


def test_both_convex_windings_and_no_input_mutation():
    for points in (FOOTPRINT, list(reversed(FOOTPRINT))):
        assert profile(footprint=points)


def test_nested_custom_node_clock_is_overridden(tmp_path):
    data = yaml.safe_load(BASE.read_text())
    data['custom'] = {'child': {'ros__parameters': {'use_sim_time': False}}}
    base = tmp_path / 'params.yaml'
    base.write_text(yaml.safe_dump(data))
    p = build_params(base, OVERRIDES, 'sim')
    assert p['custom']['child']['ros__parameters']['use_sim_time'] is True
    assert yaml.safe_load(base.read_text()) == data


def test_sim_initial_pose_matches_return_home():
    initial = profile()['amcl']['ros__parameters']['initial_pose']
    places = yaml.safe_load((ROOT / 'config/places_sim.yaml').read_text())
    assert places['places']['home']['pose'] == [initial['x'], initial['y'], initial['yaw']]


def test_precision_trial_is_sim_only():
    sim = profile()
    real = profile('real', footprint=FOOTPRINT, max_linear=.12, max_angular=.15)
    for params, xy, yaw, planner in ((sim, .03, .08, .02), (real, .06, .15, .1)):
        checker = params['controller_server']['ros__parameters']['general_goal_checker']
        assert checker['xy_goal_tolerance'] == xy
        assert checker['yaw_goal_tolerance'] == yaw
        assert params['planner_server']['ros__parameters']['GridBased']['tolerance'] == planner
    assert sim['controller_server']['ros__parameters']['FollowPath'][
        'min_approach_linear_velocity'] == .02


def test_range_layers_are_sim_only_and_before_inflation():
    for mode, kwargs in [('sim', {}), ('real', dict(
            footprint=FOOTPRINT, max_linear=.12, max_angular=.15))]:
        p = profile(mode, **kwargs)
        for name in ('local_costmap', 'global_costmap'):
            c = p[name][name]['ros__parameters']
            assert ('sonar_layer' in c['plugins']) is (mode == 'sim')
            if mode == 'sim':
                assert c['plugins'][-2:] == ['sonar_layer', 'inflation_layer']
                assert c['sonar_layer']['plugin'] == 'nav2_costmap_2d::ObstacleLayer'
                source = c['sonar_layer']['sonar_proxy']
                assert source['topic'] == '/sim/sonar_scan'
                assert source['marking'] and source['clearing'] and source['inf_is_valid']
                assert source['expected_update_rate'] == .5
                assert source['obstacle_max_range'] < source['raytrace_max_range'] == .95


def test_standalone_nav2_uses_the_trial_tracking_policy():
    from roport_pinky.navigation_policy import RPP_TRACKING
    args = dict(footprint=FOOTPRINT, max_linear=.08, max_angular=.4)
    for mode in ('sim', 'real'):
        follower = profile(mode, **args)['controller_server']['ros__parameters']['FollowPath']
        assert {key: follower[key] for key in RPP_TRACKING} == RPP_TRACKING
