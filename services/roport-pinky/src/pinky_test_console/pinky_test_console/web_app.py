# Derived from Pinky Pro (https://github.com/pinklab-art/pinky_pro,
#   commit 014a09f289e6988894fbffde2624fdb1e257815b)
#   original file: pinky_navigation/scripts/nav2_web_server.py (Flask routes part)
# Original work licensed under the Apache License, Version 2.0.
# Modified for pinky_test_console (2026): routes split out of the ROS node,
#   teleop/estop/status/map-list/IR record/marker check APIs added, JSON input validation,
#   static files restricted to the web/ folder, security headers.
# See LICENSE in this package for the full license text.
"""Flask 라우트. ROS 를 직접 import 하지 않고 node 객체의 메서드만 호출한다.

node 는 web_server.ConsoleNode (테스트에서는 가짜 객체)이다.
"""

import math
import os

from flask import Flask, Response, abort, jsonify, request, send_from_directory

from .amcl_check import AmclCheckError
from .fleet import FleetError
from .ir_record import IrRecordError
from .map_store import MapNameError
from .marker_detect import MarkerError
from .route_view import RouteCommandError, parse_route_command

# web/ 폴더에서 이 파일들만 서빙 (원본은 스크립트 폴더 전체를 static 으로 노출했음)
STATIC_FILES = {'index.html', 'app.js', 'style.css', 'ir.html', 'ir.js',
                'marker.html', 'marker.js', 'amcl.html', 'amcl.js', 'route.html', 'route.js',
                'routemap.js', 'fleet.html', 'fleet.js', 'settings.html', 'settings.js'}
POSE_CM_LIMIT = 1000.0   # 실측 좌표 입력 허용 범위(cm, 시연 공간 444.5 x 221)

SECURITY_HEADERS = {
    'Content-Security-Policy': (
        "default-src 'self'; script-src 'self'; style-src 'self'; "
        "img-src 'self' data:; connect-src 'self'; frame-ancestors 'none'"),
    'X-Content-Type-Options': 'nosniff',
    'X-Frame-Options': 'DENY',
    'Referrer-Policy': 'no-referrer',
}


class BadRequest(Exception):
    pass


def _json_body():
    # Content-Type: application/json 만 허용. 다른 사이트의 단순 form/text POST(CSRF)는
    # CORS preflight 없이 이 헤더를 보낼 수 없으므로 여기서 걸러진다.
    if not request.is_json:
        raise BadRequest('Content-Type must be application/json')
    data = request.get_json(silent=True)
    if not isinstance(data, dict):
        raise BadRequest('JSON object body required')
    return data


def _number(data, key, default=None):
    if key not in data:
        if default is None:
            raise BadRequest(f"missing '{key}'")
        return default
    v = data[key]
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        raise BadRequest(f"'{key}' must be a finite number")
    return float(v)


def _pose_cm(data, need_yaw=True):
    """{x_cm, y_cm, yaw_deg} → (x, y, yaw|None). 범위 밖이면 BadRequest."""
    x, y = _number(data, 'x_cm'), _number(data, 'y_cm')
    if not (abs(x) <= POSE_CM_LIMIT and abs(y) <= POSE_CM_LIMIT):
        raise BadRequest(f'x_cm, y_cm must be within ±{POSE_CM_LIMIT:g}')
    if 'yaw_deg' not in data and not need_yaw:
        return x, y, None
    yaw = _number(data, 'yaw_deg', 0.0)
    if abs(yaw) > 360.0:
        raise BadRequest('yaw_deg must be within ±360')
    return x, y, yaw


def create_app(node, web_dir, fleet=None, allowed_hosts=None):
    """fleet: fleet.FleetManager(mode:=fleet 일 때만). allowed_hosts: Host 헤더 허용 목록.

    fleet 모드는 SSH 로 로봇에 설치·설정하므로 127.0.0.1 에만 열고, DNS rebinding 으로 다른 사이트가
    localhost 에 요청하지 못하게 Host 헤더를 검사한다(allowed_hosts).
    """
    app = Flask(__name__, static_folder=None)
    app.config['MAX_CONTENT_LENGTH'] = 16 * 1024

    @app.errorhandler(BadRequest)
    def _bad_request(e):
        return jsonify({'success': False, 'msg': str(e)}), 400

    @app.errorhandler(RouteCommandError)
    @app.errorhandler(FleetError)
    def _bad_value(e):
        return jsonify({'success': False, 'msg': str(e)}), 400

    @app.before_request
    def _check_host():
        if allowed_hosts is not None and request.host not in allowed_hosts:
            abort(403)

    @app.after_request
    def _headers(resp):
        for k, v in SECURITY_HEADERS.items():
            resp.headers.setdefault(k, v)
        if request.path.startswith('/api/'):
            resp.headers['Cache-Control'] = 'no-store'
        return resp

    # ---------------- 정적 파일 ----------------
    @app.route('/')
    def serve_index():
        return send_from_directory(web_dir, 'index.html')

    @app.route('/<path:fname>')
    def serve_static(fname):
        if fname not in STATIC_FILES:
            abort(404)
        return send_from_directory(web_dir, fname)

    # ---------------- 조회 ----------------
    @app.route('/api/info')
    def api_info():
        return jsonify(node.get_info())

    @app.route('/api/state')
    def api_state():
        """pose·경로·주행 상태 + 바뀐 레이어만 grid 데이터 포함.

        query: map_v / global_v / local_v = 브라우저가 가진 버전, layers=map,global,local
        """
        layers = set(request.args.get('layers', 'map,global,local').split(','))
        versions = {}
        for key in ('map', 'global', 'local'):
            try:
                versions[key] = int(request.args.get(key + '_v', '-1'))
            except ValueError:
                versions[key] = -1
        return jsonify(node.get_state_snapshot(versions, layers))

    @app.route('/api/status')
    def api_status():
        return jsonify(node.get_status())

    @app.route('/api/nav/status')
    def api_nav_status():
        return jsonify({'navigating': node.is_navigating()})

    @app.route('/api/maps')
    def api_maps():
        return jsonify(node.list_maps())

    # ---------------- Nav2 ----------------
    @app.route('/api/goal', methods=['POST'])
    def api_goal():
        data = _json_body()
        ok = node.send_goal(_number(data, 'x'), _number(data, 'y'), _number(data, 'yaw', 0.0))
        return jsonify({'success': ok})

    @app.route('/api/initialpose', methods=['POST'])
    def api_initialpose():
        data = _json_body()
        ok = node.set_initial_pose(
            _number(data, 'x'), _number(data, 'y'), _number(data, 'yaw', 0.0))
        return jsonify({'success': ok})

    @app.route('/api/nav/stop', methods=['POST'])
    def api_nav_stop():
        _json_body()
        return jsonify({'success': node.cancel_goal()})

    # ---------------- 조종 / 정지 ----------------
    @app.route('/api/teleop', methods=['POST'])
    def api_teleop():
        try:
            data = _json_body()
            lin = _number(data, 'linear')
            ang = _number(data, 'angular')
        except BadRequest:
            node.teleop_stop()  # 이상한 입력이 오면 움직이지 않는다
            raise
        return jsonify(dict(success=True, **node.teleop(lin, ang)))

    @app.route('/api/teleop/stop', methods=['POST'])
    def api_teleop_stop():
        _json_body()
        node.teleop_stop()
        return jsonify({'success': True})

    @app.route('/api/estop', methods=['POST'])
    def api_estop():
        _json_body()
        return jsonify(dict(success=True, **node.emergency_stop()))

    # ---------------- SLAM ----------------
    @app.route('/api/slam/reset', methods=['POST'])
    def api_slam_reset():
        _json_body()
        return jsonify({'success': node.slam_reset()})

    @app.route('/api/slam/save_map', methods=['POST'])
    def api_slam_save_map():
        data = _json_body()
        name = data.get('name', '')
        if isinstance(name, str):
            name = name.strip()
        overwrite = data.get('overwrite', False) is True
        try:
            result = node.slam_save_map(name, overwrite)
        except MapNameError as e:
            return jsonify({'success': False, 'msg': str(e)}), 400
        code = 200
        if result.get('exists'):
            code = 409
        return jsonify(result), code

    # ---------------- IR 측정 ----------------
    @app.route('/api/ir')
    def api_ir():
        return jsonify(node.ir_snapshot())

    @app.route('/api/ir/record', methods=['POST'])
    def api_ir_record():
        data = _json_body()
        label = data.get('label', '')
        if isinstance(label, str):
            label = label.strip()
        try:
            state = node.ir_start(label, _number(data, 'duration'))
        except IrRecordError as e:
            return jsonify({'success': False, 'msg': str(e)}), 400
        return jsonify({'success': True, 'recording': state})

    @app.route('/api/ir/cancel', methods=['POST'])
    def api_ir_cancel():
        _json_body()
        return jsonify({'success': node.ir_cancel()})

    @app.route('/api/ir/records')
    def api_ir_records():
        return jsonify(node.ir_records())

    @app.route('/api/ir/records/<name>')
    def api_ir_record_file(name):
        try:
            path = node.ir_record_path(name)
        except IrRecordError:
            abort(404)
        if not os.path.isfile(path):
            abort(404)
        return send_from_directory(os.path.dirname(path), os.path.basename(path),
                                   as_attachment=True, mimetype='text/csv')

    # ---------------- 마커 인식 확인 ----------------
    @app.route('/api/marker')
    def api_marker():
        return jsonify(node.marker_snapshot())

    @app.route('/api/marker/frame.jpg')
    def api_marker_frame():
        jpeg = node.marker_jpeg()
        if jpeg is None:
            return Response(status=204)
        return Response(jpeg, mimetype='image/jpeg', headers={'Cache-Control': 'no-store'})

    @app.route('/api/marker/reset', methods=['POST'])
    def api_marker_reset():
        _json_body()
        return jsonify({'success': node.marker_reset()})

    @app.route('/api/marker/record', methods=['POST'])
    def api_marker_record():
        data = _json_body()
        marker_id = data.get('marker_id')
        try:
            state = node.marker_record_start(
                _number(data, 'distance_cm'), _number(data, 'duration', 5.0),
                marker_id, data.get('note', ''))
        except MarkerError as e:
            return jsonify({'success': False, 'msg': str(e)}), 400
        return jsonify({'success': True, 'recording': state})

    @app.route('/api/marker/records')
    def api_marker_records():
        return jsonify(node.marker_records())

    @app.route('/api/marker/records.csv')
    def api_marker_records_csv():
        path = node.marker_records_path()
        if not path or not os.path.isfile(path):
            abort(404)
        return send_from_directory(os.path.dirname(path), os.path.basename(path),
                                   as_attachment=True, mimetype='text/csv')

    # ---------------- AMCL 정합 확인 ----------------
    @app.route('/api/amcl')
    def api_amcl():
        try:
            map_v = int(request.args.get('map_v', '-1'))
        except ValueError:
            map_v = -1
        return jsonify(node.amcl_snapshot(map_v))

    @app.route('/api/amcl/initialpose', methods=['POST'])
    def api_amcl_initialpose():
        """{node: 'N12', yaw_deg?} 또는 {x_cm, y_cm, yaw_deg} (실측 좌표)."""
        data = _json_body()
        try:
            if 'node' in data:
                n = node.amcl_node(data['node'])
                yaw = _number(data, 'yaw_deg') if 'yaw_deg' in data else n['yaw_deg']
                ok = node.amcl_initial_pose_cm(n['x_cm'], n['y_cm'], yaw)
            else:
                ok = node.amcl_initial_pose_cm(*_pose_cm(data))
        except AmclCheckError as e:
            return jsonify({'success': False, 'msg': str(e)}), 400
        return jsonify({'success': ok})

    @app.route('/api/amcl/nomotion', methods=['POST'])
    def api_amcl_nomotion():
        _json_body()
        return jsonify({'success': node.amcl_nomotion_update()})

    @app.route('/api/amcl/record', methods=['POST'])
    def api_amcl_record():
        """{label, truth?: {x_cm, y_cm, yaw_deg?}} — truth 는 줄자로 잰 실제 위치."""
        data = _json_body()
        truth = data.get('truth')
        if truth is not None:
            if not isinstance(truth, dict):
                raise BadRequest("'truth' must be an object")
            truth = _pose_cm(truth, need_yaw=False)
        label = data.get('label', '')
        if isinstance(label, str):
            label = label.strip()
        try:
            row = node.amcl_record(label, truth)
        except AmclCheckError as e:
            return jsonify({'success': False, 'msg': str(e)}), 400
        return jsonify({'success': True, 'row': row})

    @app.route('/api/amcl/records.csv')
    def api_amcl_records_csv():
        path = node.amcl_records_path()
        if not path or not os.path.isfile(path):
            abort(404)
        return send_from_directory(os.path.dirname(path), os.path.basename(path),
                                   as_attachment=True, mimetype='text/csv')

    # ---------------- 노드 경로 주행 (mode:=route) ----------------
    @app.route('/api/route')
    def api_route():
        return jsonify(node.route_snapshot())

    @app.route('/api/route/graph')
    def api_route_graph():
        return jsonify(node.route_graph())

    @app.route('/api/route/command', methods=['POST'])
    def api_route_command():
        cmd = parse_route_command(_json_body())
        return jsonify({'success': node.route_command(cmd), 'sent': cmd})

    # ---------------- Pinky 설정·테스트 관제 (mode:=fleet, 노트북) ----------------
    def _fleet():
        if fleet is None:
            abort(404)
        return fleet

    def _proxy(fn):
        try:
            return jsonify({'success': True, 'result': fn()})
        except OSError as e:     # 로봇 콘솔이 꺼져 있거나 네트워크가 다름
            return jsonify({'success': False, 'msg': f'robot not reachable: {e}'}), 502

    @app.route('/api/fleet')
    def api_fleet():
        return jsonify(_fleet().snapshot())

    @app.route('/api/fleet/robots', methods=['POST'])
    def api_fleet_upsert():
        robot = _fleet().store.upsert(_json_body())
        return jsonify({'success': True, 'robot': robot.__dict__})

    @app.route('/api/fleet/robots/delete', methods=['POST'])
    def api_fleet_delete():
        name = _json_body().get('name')
        if not isinstance(name, str):
            raise BadRequest("'name' required")
        _fleet().store.remove(name)
        return jsonify({'success': True})

    @app.route('/api/fleet/branches')
    def api_fleet_branches():
        return jsonify({'branches': _fleet().branches()})

    @app.route('/api/fleet/job', methods=['POST'])
    def api_fleet_job():
        data = _json_body()
        params = {}
        if 'ref' in data:
            params['ref'] = data['ref']
        if data.get('release') is True:
            params['release'] = True
        if 'domain' in data:
            params['domain'] = data['domain']
        job = _fleet().start_job(data.get('name'), data.get('action'), params)
        return jsonify({'success': True, 'job': job})

    @app.route('/api/fleet/jobs')
    def api_fleet_jobs():
        return jsonify({'jobs': _fleet().jobs()})

    @app.route('/api/fleet/route', methods=['POST'])
    def api_fleet_route():
        data = _json_body()
        name = data.pop('name', None)
        if not isinstance(name, str):
            raise BadRequest("'name' required")
        f = _fleet()
        return _proxy(lambda: f.route_command(name, data))

    @app.route('/api/fleet/stop_all', methods=['POST'])
    def api_fleet_stop_all():
        _json_body()
        return jsonify({'success': True, 'result': _fleet().stop_all()})

    return app


def default_web_dir():
    """설치된 share/pinky_test_console/web, 없으면 소스 트리의 web/."""
    try:
        from ament_index_python.packages import get_package_share_directory
        d = os.path.join(get_package_share_directory('pinky_test_console'), 'web')
        if os.path.isdir(d):
            return d
    except Exception:  # noqa: BLE001 - 패키지 미설치(소스 실행/테스트)
        pass
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'web')
