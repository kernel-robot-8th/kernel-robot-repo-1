"""Supervised standalone trial using one deployed hardware-return version.

Nav2 drives a surveyed gate; supervised vision corrects only the final distance.
This trial does not claim continuous visual pose correction. Return requires a
delivery-complete command matching this exact mission, and physical IR at N12.
"""
import argparse
import os
import hashlib
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import time
from types import SimpleNamespace
from uuid import uuid4

import cv2
import numpy as np
import yaml

ROOT = Path(os.environ['ROPORT_FIELD_CONFIG_DIR']).expanduser().resolve()
class DockMarkerLost(RuntimeError):
    pass


TARGET = json.loads((ROOT/'mission-target.json').read_text()) if (ROOT/'mission-target.json').exists() else 'G1'
GOAL, MARKER_ID = {'G1': ('gate1', 8), 'G2': ('gate2', 9), 'G3': ('gate3', 10), 'ARR': ('arrival', 27)}[TARGET]


def centered(reading, age):
    return bool(reading and 0 <= age <= .3 and not reading.lost
                and reading.on_line[1] and math.isfinite(reading.error)
                and abs(reading.error) <= .25)


def valid_delivery(packet, mission_id):
    return (isinstance(packet, dict) and packet.get('mission_id') == mission_id
            and packet.get('command') == 'delivery_complete')



def confirmed_gate_return(state, gate):
    pose = state.get('pose')
    return bool(state.get('nav2_ready') and state.get('localization_ready')
        and state.get('stopped') and pose
        and all(math.isfinite(pose[k]) for k in ('x', 'y', 'yaw'))
        and math.hypot(pose['x']-gate.x, pose['y']-gate.y) <= .05
        and abs(math.atan2(math.sin(pose['yaw']-gate.yaw), math.cos(pose['yaw']-gate.yaw))) <= .1)



def stopped_checkpoint(previous, samples):
    pose = previous.get('final_state', {}).get('pose')
    if pose and previous.get('final_state', {}).get('stopped'):
        return pose
    for row in reversed(samples):
        if (row.get('mission_id') == previous.get('mission_id')
                and row.get('direction') == 'RETURN' and row.get('stopped')
                and row.get('localization_ready') and row.get('pose')):
            return row['pose']
    raise RuntimeError('No actual stopped return observation')


def reverse_to_search_start(pose, previous, samples):
    candidates = [row for row in samples if row.get('mission_id') == previous.get('mission_id')
        and row.get('phase') == 'NAV2_RETURN_N12' and row.get('stopped')
        and row.get('localization_ready') and row.get('pose')]
    if not candidates:
        raise RuntimeError('No recorded N12 search-start pose')
    target = candidates[-1]['pose']
    dx, dy = target['x']-pose['x'], target['y']-pose['y']
    backward = -(dx*math.cos(pose['yaw'])+dy*math.sin(pose['yaw']))
    lateral = abs(-dx*math.sin(pose['yaw'])+dy*math.cos(pose['yaw']))
    angle = abs(math.atan2(math.sin(target['yaw']-pose['yaw']), math.cos(target['yaw']-pose['yaw'])))
    if not 0 < backward <= .07 or lateral > .015 or angle > .1:
        raise RuntimeError('Recorded N12 search rollback outside bounds')
    return backward


def main(resume_nav=False, resume_entry=False, resume_return=False, resume_return_transit=False, resume_delivery=False, resume_n12=False):
    import os
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.parameter_client import AsyncParameterClient
    from rclpy.parameter import Parameter
    from ament_index_python.packages import get_package_share_directory
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String
    from std_srvs.srv import Empty
    from action_msgs.msg import GoalStatus
    from nav2_msgs.action import BackUp, Spin, DriveOnHeading
    from roport_smartcart.real_return import HardwareReturn, n12_target, measured_handoff
    from roport_smartcart.goals import Destination, load_destination
    from roport_gz_world.layout import Layout
    from roport_line_nav.graph import Graph
    from roport_pinky_line.calibration import load_calibration
    if os.environ.get('ROS_DOMAIN_ID') != '13':
        raise RuntimeError('ROS domain must be 13')
    options = SimpleNamespace(map=os.environ['ROPORT_PINKY_MAP'],
        layout=str(ROOT/'layout.yaml'), robot=os.environ['ROBOT_NAME'],
        places=str(ROOT/'field-places.yaml'), bay='H1', line_speed=.06, from_n12=True)
    graph = Graph.load(options.layout)
    layout = Layout(yaml.safe_load(Path(options.layout).read_text()))
    line_cal = load_calibration(str(Path(get_package_share_directory('roport_pinky_line'))/
                                   'config/robots.yaml'), options.robot)
    nominal = n12_target(layout, graph)
    target = measured_handoff(load_destination(options.places, options.map,
                                              'return_n12', environment='real'), nominal)
    for name in ('outbound_entry', GOAL, 'return_entry', 'return_n12'):
        load_destination(options.places, options.map, name, environment='real')
    path = Path(options.map)
    map_data = yaml.safe_load(path.read_text())
    profile = {'map_id': path.stem, 'map_version': hashlib.sha256(path.read_bytes()+
        (path.parent/map_data['image']).read_bytes()).hexdigest()[:16]}
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('supervised_h1_g3_ui_trial')
    runner = HardwareReturn(node, options, graph, line_cal, target, profile)
    mission_id = uuid4().hex
    progress = node.create_publisher(String, 'smartcart/roundtrip_state', 1)
    refresh = node.create_client(Empty, 'request_nomotion_update')
    parameters = AsyncParameterClient(node, 'controller_server')
    camera = {}
    subscription = node.create_subscription(CompressedImage, '/camera/image_raw/compressed',
        lambda m: camera.update(message=m), qos_profile_sensor_data)
    original_spin = runner.spin
    rows, marker_samples = [], []
    last_status = last_refresh = 0.
    direction = 'OUTBOUND'
    started = time.monotonic()
    action_client = handle = sending = None
    saved_parameters = None
    success = False
    failure = None
    marker_verified = False
    arrival_warning = None
    camera_process = None
    camera_log = None

    # Translate the confirmed-start semantics to the installed executor API.
    original_start = runner.line.start
    def line_start(route, **kwargs):
        confirmed = kwargs.pop('confirmed_start', False)
        if confirmed and route[0] != 'N12':
            raise RuntimeError('Unexpected confirmed route start')
        original_start(route, **kwargs)
        if route[0] == 'N12':
            runner.line._begin_leg(0., guess=False)
    runner.line.start = line_start

    def spin():
        nonlocal last_status
        for _ in range(8):
            rclpy.spin_once(node, timeout_sec=0.)
        state = original_spin()
        now = time.monotonic()
        if now-last_status >= .2:
            packet = {'mission_id': mission_id, 'phase': runner.phase,
                'direction': direction, 'goal': TARGET, 'elapsed_s': now-started,
                'operator_return_resume': resume_return,
                'stopped': state['stopped'], 'pose': state.get('pose'),
                'localization_ready': state['localization_ready'],
                'line_index': runner.line.index, 'route': runner.line.route,
                'marker': marker_samples[-1] if marker_verified and marker_samples else None,
                'marker_verified': marker_verified, 'arrival_warning': arrival_warning,
                'error': failure, 'sent_at': time.time()}
            progress.publish(String(data=json.dumps(packet)))
            rows.append({**packet, 'ir': runner.reading.as_dict() if runner.reading else None})
            last_status = now
        return state
    runner.spin = spin

    def wait(future, seconds=6, require_guard=True):
        end = time.monotonic()+seconds
        while not future.done():
            spin()
            if require_guard:
                runner.require_guard()
            if time.monotonic() >= end:
                raise RuntimeError('Action/service response timed out')
        result = future.result()
        if result is None:
            raise RuntimeError('Empty action/service response')
        return result

    def settle(seconds=15):
        nonlocal last_refresh
        runner.owner = 'none'
        runner.phase = 'LOCALIZATION_SETTLE'
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            state = spin()
            runner.require_guard()
            now = time.monotonic()
            if now-last_refresh > .4 and refresh.service_is_ready():
                refresh.call_async(Empty.Request())
                last_refresh = now
            amcl = runner.bridge.amcl
            covariance = amcl.pose.covariance if amcl else []
            if (state['nav2_ready'] and state['localization_ready'] and state['stopped']
                    and len(covariance) == 36 and all(covariance[i] <= limit for i, limit in
                                           ((0, .01), (7, .01), (35, .04)))):
                return state
        raise RuntimeError('Localization did not settle while stopped')

    def navigate(place):
        for attempt in range(2):
            settle()
            try:
                runner.navigate(place)
                return
            except RuntimeError as exc:
                if attempt or str(exc) != 'Nav2/localization readiness lost':
                    raise
                runner.owner = 'none'
                runner.lease('none')
                runner.nav.cancel()
                end = time.monotonic()+10
                while runner.nav.lifecycle.busy:
                    spin()
                    if time.monotonic() > end:
                        raise RuntimeError('Previous Nav2 goal termination unknown')
                if runner.nav.lifecycle.fault:
                    raise RuntimeError('Previous Nav2 transport failed')
                runner.stop_confirmed()
                print('LOCALIZATION PAUSE: stopped, terminal goal confirmed; one bounded retry', flush=True)

    def behavior(kind, name, goal, seconds, stop_on_line=False, marker_watch=False):
        nonlocal action_client, handle, sending
        runner.stop_confirmed()
        runner.phase = name.upper()
        action_client = ActionClient(node, kind, name)
        if not action_client.wait_for_server(timeout_sec=2):
            raise RuntimeError(name+' server unavailable')
        sending = action_client.send_goal_async(goal)
        handle = wait(sending, 3)
        if not handle.accepted:
            raise RuntimeError(name+' rejected')
        future = handle.get_result_async()
        runner.owner = 'nav2'
        end = time.monotonic()+seconds
        found = False
        while not future.done():
            state = spin()
            runner.require_guard()
            if not state['localization_ready'] or not state['nav2_ready']:
                raise RuntimeError(name+' localization unavailable')
            if marker_watch:
                observation = observe_marker()
                if observation is None or not observation['safe']:
                    runner.owner = 'none'
                    runner.lease('none')
                    wait(handle.cancel_goal_async())
                    wait(future)
                    runner.stop_confirmed()
                    handle = sending = None
                    action_client.destroy()
                    action_client = None
                    raise DockMarkerLost('Docking marker lost/invalid; approach canceled')
            if stop_on_line:
                age = float('inf') if runner.ir_at is None else time.monotonic()-runner.ir_at
                if age > .3:
                    raise RuntimeError('Physical IR stale during line acquisition')
                if centered(runner.reading, age):
                    found = True
                    runner.owner = 'none'
                    runner.lease('none')
                    wait(handle.cancel_goal_async())
                    break
            if time.monotonic() > end:
                raise RuntimeError(name+' execution timeout')
        runner.owner = 'none'
        runner.lease('none')
        result = wait(future)
        state = runner.stop_confirmed()
        if result.status != GoalStatus.STATUS_SUCCEEDED and not (
                stop_on_line and found and result.status == GoalStatus.STATUS_CANCELED):
            raise RuntimeError(name+' failed: '+str(result.result))
        handle = sending = None
        action_client.destroy()
        action_client = None
        return state, found

    def start_gate_camera():
        nonlocal camera_process, camera_log
        if node.count_publishers('/camera/image_raw/compressed'):
            return
        camera_log = (ROOT/'camera.log').open('ab')
        camera_process = subprocess.Popen([
            sys.executable,
            str(Path(__file__).with_name('camera_stream.py'))],
            stdout=camera_log, stderr=camera_log)

    def stop_gate_camera():
        nonlocal camera_process, camera_log
        if camera_process is not None:
            camera_process.terminate()
            try:
                camera_process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                camera_process.kill()
                camera_process.wait(timeout=2)
            camera_process = None
        if camera_log is not None:
            camera_log.close()
            camera_log = None

    def observe_marker():
        msg = camera.get('message')
        if msg is None:
            return None
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        if not -.05 <= node.get_clock().now().nanoseconds/1e9-stamp <= .3:
            return None
        image = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_GRAYSCALE)
        if image is None or image.shape != (480, 640):
            return None
        cfg = yaml.safe_load((ROOT/'camera_calibration.yaml').read_text())
        k = np.array(cfg['camera_matrix']['data']).reshape(3,3)
        d = np.array(cfg['distortion_coefficients']['data'])
        obj = np.array([[-.03,.03,0],[.03,.03,0],[.03,-.03,0],[-.03,-.03,0]],np.float32)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        detector = cv2.aruco.ArucoDetector(dictionary) if hasattr(cv2.aruco,'ArucoDetector') else None
        corners, ids, _ = detector.detectMarkers(image) if detector else cv2.aruco.detectMarkers(image,dictionary)
        indexes = [] if ids is None else [i for i,v in enumerate(ids.flatten()) if v == MARKER_ID]
        if len(indexes) != 1:
            return None
        points = corners[indexes[0]].reshape(4,2)
        ok, rv, tv = cv2.solvePnP(obj,points,k,d,flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok or not np.isfinite(tv).all() or tv[2,0] <= 0:
            return None
        normal = cv2.Rodrigues(rv)[0][:,2]
        gap = float(abs(normal @ tv[:,0]))
        projected,_ = cv2.projectPoints(obj,rv,tv,k,d)
        error = float(np.sqrt(np.mean(np.sum((projected.reshape(4,2)-points)**2,axis=1))))
        bearing = float(math.atan2(tv[0,0],tv[2,0]))
        return {'id': MARKER_ID, 'stamp': stamp, 'lens_plane_cm': gap*100,
                'reprojection_px': error, 'bearing_rad': bearing,
                'safe': .12 <= gap <= .3 and error <= 2 and abs(bearing) <= .2}

    def confirm_gate(allow_approach=True):
        runner.phase = TARGET+'_MARKER_CHECK'
        runner.stop_confirmed()
        reference_path = ROOT/'gate-distance-reference.json'
        reference = json.loads(reference_path.read_text()) if reference_path.exists() else None
        if reference is None:
            return False
        if reference is not None:
            if (reference['marker_id'] != MARKER_ID or reference['map_version'] != profile['map_version']
                    or reference['calibration_sha256'] != hashlib.sha256((ROOT/'camera_calibration.yaml').read_bytes()).hexdigest()
                    or not math.isfinite(reference['front_minus_lens_m'])
                    or abs(reference['front_minus_lens_m']) > .03):
                raise RuntimeError('Dock distance reference mismatch')
        end, previous, stable, steps = time.monotonic()+45, None, [], 0
        while time.monotonic() < end:
            state = spin()
            runner.require_guard()
            if not state['stopped']:
                raise RuntimeError(TARGET+' standstill lost')
            observation = observe_marker()
            if observation is None or not observation['safe']:
                stable.clear()
                continue
            if observation['stamp'] == previous:
                continue
            previous = observation['stamp']
            if stable and abs(observation['lens_plane_cm']-stable[-1]['lens_plane_cm']) > 1:
                stable.clear()
            stable = (stable+[observation])[-5:]
            if len(stable) < 5:
                continue
            gap = float(np.median([v['lens_plane_cm'] for v in stable]))/100
            front_gap = gap + (reference['front_minus_lens_m'] if reference else 0.)
            if .13 <= front_gap <= .17:
                marker_samples[:] = stable
                print(TARGET+' stopped marker distance confirmed (front reference; angular docking not claimed)',flush=True)
                return True
            if not allow_approach or reference is None or not .17 < front_gap <= .25 or steps >= 6:
                break
            distance = min(.01, front_gap-.155)
            print('MARKER_APPROACH',json.dumps({'step_m':distance,'front_gap_m':front_gap}),flush=True)
            goal = DriveOnHeading.Goal()
            goal.target.x, goal.speed, goal.time_allowance.sec = distance, .01, 4
            try:
                behavior(DriveOnHeading,'drive_on_heading',goal,6,marker_watch=True)
            except DockMarkerLost:
                marker_samples.clear()
                return False
            steps += 1
            runner.phase = TARGET+'_MARKER_CHECK'
            stable.clear()
        marker_samples.clear()
        return False

    def realign_n12():
        target_pose = load_destination(options.places, options.map, 'return_n12', environment='real')
        state = runner.stop_confirmed()
        pose = state.get('pose')
        if not state['nav2_ready'] or not state['localization_ready'] or not pose:
            raise RuntimeError('N12 realignment readiness unavailable')
        dx, dy = target_pose.x-pose['x'], target_pose.y-pose['y']
        distance = math.hypot(dx, dy)
        if not state['localization_ready'] or distance > .08:
            raise RuntimeError('N12 realignment exceeds 8cm bound or localization unavailable')
        print('N12_REALIGN',json.dumps({'dx':dx,'dy':dy,'distance':distance}),flush=True)
        if distance > .005:
            heading = math.atan2(dy,dx)
            turn = Spin.Goal()
            turn.target_yaw = math.atan2(math.sin(heading-pose['yaw']),math.cos(heading-pose['yaw']))
            turn.time_allowance.sec = 20
            behavior(Spin,'spin',turn,23)
            step = DriveOnHeading.Goal()
            step.target.x, step.speed, step.time_allowance.sec = distance, .01, 12
            behavior(DriveOnHeading,'drive_on_heading',step,15)
        state = runner.stop_confirmed()
        turn = Spin.Goal()
        turn.target_yaw = math.atan2(math.sin(target_pose.yaw-state['pose']['yaw']),math.cos(target_pose.yaw-state['pose']['yaw']))
        turn.time_allowance.sec = 20
        behavior(Spin,'spin',turn,23)

    def interrupt(*_):
        raise KeyboardInterrupt()
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, interrupt)
    try:
        end = time.monotonic()+30
        while time.monotonic() < end:
            state = spin()
            if runner.boot is None and runner.guard:
                runner.boot = runner.guard.get('boot')
            if runner.guard_ok() and state['nav2_ready'] and state['localization_ready'] and state['stopped']:
                break
        else:
            raise RuntimeError('Start readiness failed')
        for topic, name in (('cmd_vel', 'trial_guard'), ('trial/lease', node.get_name()),
                            ('line_nav/cmd_vel', node.get_name())):
            discovery_end = time.monotonic()+5
            while True:
                pubs = node.get_publishers_info_by_topic(topic)
                if len(pubs) == 1 and pubs[0].node_name == name:
                    break
                if time.monotonic() >= discovery_end:
                    break
                spin()
            if len(pubs) != 1 or pubs[0].node_name != name:
                raise RuntimeError('Publisher conflict: '+topic)
        if runner.guard.get('owner') != 'none':
            raise RuntimeError('Drive already owned')
        state = settle()
        if resume_return_transit or resume_n12:
            previous = json.loads((ROOT/'result.json').read_text())
            if (previous.get('phase') != 'STOPPED' or not (
                    (resume_n12 and (any(text in (previous.get('failure') or '') for text in ('Physical tape', 'N12 realignment')) or previous.get('scope','').startswith('Operator-authorized return')))
                    or (not resume_n12 and previous.get('scope','').startswith('Operator-authorized return')))):
                raise RuntimeError('No matching stopped return checkpoint')
            previous_samples = json.loads((ROOT/'samples.json').read_text())
            boot_time = time.time()-float(Path('/proc/uptime').read_text().split()[0])
            if (ROOT/'result.json').stat().st_mtime < boot_time-1:
                raise RuntimeError('Return checkpoint predates this robot boot')
            saved = stopped_checkpoint(previous, previous_samples)
            checkpoint = Destination(saved['x'], saved['y'], saved['yaw'])
            if not confirmed_gate_return(state, checkpoint):
                raise RuntimeError('Current stopped pose differs from return checkpoint by over 5cm/0.1rad')
        elif resume_return or resume_delivery:
            gate = load_destination(options.places, options.map, GOAL, environment="real")
            if not confirmed_gate_return(state, gate):
                raise RuntimeError("Return start requires fresh stopped surveyed gate pose within 5cm/0.1rad")
        if not resume_nav and not resume_return and not resume_delivery:
            expected = json.loads((ROOT/'h1-confirmed.json').read_text())['final_state']['pose']
            pose = state['pose']
            if (math.hypot(pose['x']-expected['x'], pose['y']-expected['y']) > .10
                    or abs(math.atan2(math.sin(pose['yaw']-expected['yaw']),
                                      math.cos(pose['yaw']-expected['yaw']))) > .25
                    or runner.ir_at is None or not centered(runner.reading, time.monotonic()-runner.ir_at)):
                raise RuntimeError('H1 physical placement/heading/IR not confirmed')
        old = wait(parameters.get_parameters(['general_goal_checker.xy_goal_tolerance',
                                              'general_goal_checker.yaw_goal_tolerance']))
        if len(old.values) != 2 or any(v.type != 3 for v in old.values):
            raise RuntimeError('Goal tolerance parameters unavailable')
        saved_parameters = [v.double_value for v in old.values]
        updated = wait(parameters.set_parameters([
            Parameter('general_goal_checker.xy_goal_tolerance', value=.05),
            Parameter('general_goal_checker.yaw_goal_tolerance', value=.1)]))
        if not all(v.successful for v in updated.results):
            raise RuntimeError('Goal tolerance update rejected')
        if not resume_return:
            if not resume_delivery:
                if not resume_nav:
                    print('START H1 physical line -> N09 -> N08 -> N12', flush=True)
                    runner.follow_line(['H1', 'N09', 'N08', 'N12'])
                    navigate('outbound_entry')
                else:
                    print('RESUME Nav2 from stopped field pose', flush=True)
                    if resume_entry:
                        navigate('outbound_entry')
                navigate(GOAL)
            try:
                start_gate_camera()
            except OSError as exc:
                arrival_warning = 'Camera startup failed: '+str(exc)
            marker_verified = confirm_gate(allow_approach=not resume_delivery)
            if not marker_verified:
                arrival_warning = arrival_warning or (TARGET+' marker/range unconfirmed; coordinate arrival only')
                print('WARNING', arrival_warning, flush=True)
            runner.stop_confirmed()
            runner.phase = 'WAIT_DELIVERY'
            print('WAIT_DELIVERY: only matching UI completion command starts return', flush=True)
            end = time.monotonic()+1800
            while time.monotonic() < end:
                state = spin()
                runner.require_guard()
                if not state['stopped'] or runner.nav.lifecycle.busy:
                    raise RuntimeError('Delivery wait standstill/goal ownership lost')
                packet_path = ROOT/'delivery-command.json'
                try:
                    packet = json.loads(packet_path.read_text())
                except (OSError, ValueError):
                    packet = None
                if valid_delivery(packet, mission_id):
                    break
            else:
                raise RuntimeError('Delivery wait expired; remaining stopped')
        else:
            print('OPERATOR RETURN: surveyed gate pose/standstill verified; marker docking NOT claimed', flush=True)
        direction = 'RETURN'
        if not resume_return_transit and not resume_n12:
            back = BackUp.Goal()
            back.target.x, back.speed, back.time_allowance.sec = -.08, .02, 12
            behavior(BackUp, 'backup', back, 15)
            turn = Spin.Goal()
            turn.target_yaw, turn.time_allowance.sec = math.pi, 25
            behavior(Spin, 'spin', turn, 28)
        if not resume_n12:
            navigate('return_entry')
            navigate('return_n12')
        state = runner.stop_confirmed()
        age = float('inf') if runner.ir_at is None else time.monotonic()-runner.ir_at
        if not centered(runner.reading, age):
            if resume_n12:
                target_pose = load_destination(options.places, options.map, 'return_n12', environment='real')
                pose = state['pose']
                if math.hypot(pose['x']-target_pose.x, pose['y']-target_pose.y) > .08:
                    distance = reverse_to_search_start(pose, previous, previous_samples)
                    back = BackUp.Goal()
                    back.target.x, back.speed, back.time_allowance.sec = -distance, .01, 12
                    behavior(BackUp, 'backup', back, 15)
            # Align before searching, so a lateral miss cannot drive past the 8cm envelope.
            realign_n12()
            state = runner.stop_confirmed()
            age = float('inf') if runner.ir_at is None else time.monotonic()-runner.ir_at
            if not centered(runner.reading, age):
                goal = DriveOnHeading.Goal()
                goal.target.x, goal.speed, goal.time_allowance.sec = .06, .01, 12
                state, found = behavior(DriveOnHeading, 'drive_on_heading', goal, 14, stop_on_line=True)
                if not found:
                    raise RuntimeError('Physical tape not found within 6cm after bounded realignment; remaining stopped')
        pose = state['pose']
        if not state['localization_ready'] or not pose:
            raise RuntimeError('Acquired N12 pose unavailable')
        runner.target = measured_handoff(Destination(pose['x'], pose['y'], pose['yaw']), nominal)
        runner.options.from_n12 = True
        runner.run()  # fresh centered IR stable for 0.5 s, then actual line to H1
        success = runner.phase == 'ARRIVED'
    except BaseException as exc:
        failure = f'{type(exc).__name__}: {exc}'
        print('STOP', failure, flush=True)
        import traceback
        traceback.print_exc()
    finally:
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(sig, signal.SIG_IGN)
        runner.owner = 'none'
        runner.lease('none')
        try:
            if sending is not None and handle is None:
                handle = wait(sending, require_guard=False)
            if handle is not None and handle.accepted:
                wait(handle.cancel_goal_async(), require_guard=False)
                wait(handle.get_result_async(), require_guard=False)
            runner.close()
            if saved_parameters is not None:
                wait(parameters.set_parameters([
                    Parameter('general_goal_checker.xy_goal_tolerance', value=saved_parameters[0]),
                    Parameter('general_goal_checker.yaw_goal_tolerance', value=saved_parameters[1])
                ]), require_guard=False)
        except BaseException as exc:
            failure = (failure or '')+'; cleanup: '+str(exc)
            success = False
        stop_gate_camera()
        runner.phase = 'DONE' if success and failure is None else 'STOPPED'
        state = runner.bridge.snapshot()
        final = {'mission_id': mission_id, 'success': success and failure is None,
                 'failure': failure, 'phase': runner.phase, 'final_state': state,
                 'marker_verified': marker_verified, 'arrival_warning': arrival_warning,
                 'scope': ('Operator-authorized return from validated stopped checkpoint; no marker docking claim' if resume_return else 'Surveyed gate pose + stopped marker attempt (see marker_verified) + explicit delivery release + IR line H1 return')}
        (ROOT/'result.json').write_text(json.dumps(final, indent=2))
        (ROOT/'samples.json').write_text(json.dumps(rows))
        last_status = 0.
        for _ in range(4):
            spin()
        print('RESULT', json.dumps(final), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if success and failure is None else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--drive', action='store_true')
    parser.add_argument('--resume-nav', action='store_true')
    parser.add_argument('--resume-entry', action='store_true')
    parser.add_argument('--resume-return', action='store_true')
    parser.add_argument('--resume-return-transit', action='store_true')
    parser.add_argument('--resume-delivery', action='store_true')
    parser.add_argument('--resume-n12', action='store_true')
    args = parser.parse_args()
    if not args.drive:
        parser.error('--drive required after onsite authorization')
    if args.resume_n12 and (args.resume_nav or args.resume_entry or args.resume_return or args.resume_return_transit or args.resume_delivery):
        parser.error('N12 resume cannot combine with other resumes')
    if args.resume_delivery and (args.resume_nav or args.resume_entry or args.resume_return or args.resume_return_transit):
        parser.error('delivery wait resume cannot combine with movement resume')
    if (args.resume_return or args.resume_return_transit) and (args.resume_nav or args.resume_entry):
        parser.error('return resume cannot combine with outbound resume')
    raise SystemExit(main(resume_nav=args.resume_nav or args.resume_entry, resume_entry=args.resume_entry, resume_return=args.resume_return or args.resume_return_transit or args.resume_n12, resume_return_transit=args.resume_return_transit, resume_delivery=args.resume_delivery, resume_n12=args.resume_n12))
