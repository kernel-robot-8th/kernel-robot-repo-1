"""Shared Nav2 configuration; sim defaults never authorize hardware movement."""

import json
import math

from .goals import _read_yaml


def _merge(base, changes):
    for key, value in changes.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _merge(base[key], value)
        else:
            base[key] = value


def _clock_mode(tree, simulated):
    for key, value in tree.items():
        if key == 'ros__parameters' and isinstance(value, dict):
            value['use_sim_time'] = simulated
        elif isinstance(value, dict):
            _clock_mode(value, simulated)


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


def _polygon(points):
    if not isinstance(points, (list, tuple)) or len(points) < 3:
        raise ValueError('footprint must contain at least three polygon vertices')
    if not all(isinstance(p, (list, tuple)) and len(p) == 2
               and all(_number(v) for v in p) for p in points):
        raise ValueError('footprint vertices must contain finite x/y numbers')
    if len({tuple(p) for p in points}) != len(points):
        raise ValueError('footprint vertices must be distinct')
    # Each directed edge must support the whole polygon, excluding stars,
    # self-intersections and collinear/concave polygons in either winding.
    winding = None
    for index, a in enumerate(points):
        b = points[(index + 1) % len(points)]
        for c in points:
            if c == a or c == b:
                continue
            cross = (b[0] - a[0]) * (c[1] - a[1]) - (
                b[1] - a[1]) * (c[0] - a[0])
            if not math.isfinite(cross) or abs(cross) <= 1.e-12:
                raise ValueError('footprint must be strictly convex with nonzero area')
            sign = cross > 0
            if winding is not None and winding != sign:
                raise ValueError('footprint must be a simple convex polygon')
            winding = sign
    return json.dumps(points)


def build_params(base_params_file, overrides_file, mode, *, footprint=None,
                 max_linear=None, max_angular=None):
    """Return a fresh merged mapping without writing files or starting ROS.

    Sim's 16 cm square encloses the unloaded model's wheel collision spheres
    (y +/-0.07655 m) and rear body/caster; it is not a measured loaded cart.
    Real mode requires caller-supplied footprint and speed limits, which still
    need field approval. Speed caps do not replace the final velocity owner,
    collision monitor, or driver watchdog. Units: metres, seconds, radians.
    """
    if mode not in ('sim', 'real'):
        raise ValueError('mode must be sim or real')
    if mode == 'real' and any(v is None for v in (
            footprint, max_linear, max_angular)):
        raise ValueError('real mode requires explicit footprint and both speed limits')
    if footprint is None:
        footprint = [[.08, .08], [.08, -.08], [-.08, -.08], [-.08, .08]]
    polygon = _polygon(footprint)
    max_linear = .08 if max_linear is None else max_linear
    max_angular = .5 if max_angular is None else max_angular
    if not all(_number(v) and v > 0 for v in (max_linear, max_angular)):
        raise ValueError('speed limits must be finite positive numbers')

    params = _read_yaml(base_params_file)
    _merge(params, _read_yaml(overrides_file))
    simulated = mode == 'sim'
    params.setdefault('map_server', {'ros__parameters': {}})
    _clock_mode(params, simulated)
    amcl = params['amcl']['ros__parameters']
    amcl['set_initial_pose'] = simulated
    if simulated:
        # Keep the unloaded model clear of walls while turning at startup.
        amcl['initial_pose'] = {'x': 0.3, 'y': -0.3, 'z': 0.0, 'yaw': 0.0}
    else:
        amcl.pop('initial_pose', None)
    for name in ('local_costmap', 'global_costmap'):
        costmap = params[name][name]['ros__parameters']
        costmap.pop('robot_radius', None)
        costmap['footprint'] = polygon
        if simulated:
            costmap['plugins'].insert(-1, 'sonar_layer')
            costmap['sonar_layer'] = {
                'plugin': 'nav2_costmap_2d::RangeSensorLayer',
                'topics': ['/us_sensor/range'], 'input_sensor_type': 'VARIABLE',
                'clear_on_max_reading': True, 'no_readings_timeout': .5,
            }

    if simulated:
        # Trial closer, more precise stops without changing the hardware profile.
        checker = params['controller_server']['ros__parameters']['general_goal_checker']
        checker.update(xy_goal_tolerance=.03, yaw_goal_tolerance=.08)
        params['planner_server']['ros__parameters']['GridBased']['tolerance'] = .02

    follower = params['controller_server']['ros__parameters']['FollowPath']
    follower['desired_linear_vel'] = max_linear
    follower['rotate_to_heading_angular_vel'] = max_angular
    for name in ('min_approach_linear_velocity', 'regulated_linear_scaling_min_speed'):
        follower[name] = min(follower.get(name, .05), max_linear)
    if simulated:
        follower['min_approach_linear_velocity'] = min(.02, max_linear)
    behavior = params['behavior_server']['ros__parameters']
    behavior['max_rotational_vel'] = max_angular
    behavior['min_rotational_vel'] = min(behavior.get('min_rotational_vel', .4), max_angular)
    smoother = params['velocity_smoother']['ros__parameters']
    smoother['max_velocity'] = [max_linear, 0.0, max_angular]
    smoother['min_velocity'] = [-max_linear, 0.0, -max_angular]
    return params
