# Derived from Pinky Pro (https://github.com/pinklab-art/pinky_pro,
#   commit 014a09f289e6988894fbffde2624fdb1e257815b)
#   original file: pinky_navigation/scripts/nav2_web_server.py (Flask routes part)
# Original work licensed under the Apache License, Version 2.0.
# Modified for pinky_test_console (2026): routes split out of the ROS node,
#   teleop/estop/status/map-list/IR record APIs added, JSON input validation,
#   static files restricted to the web/ folder, security headers.
# See LICENSE in this package for the full license text.
"""Flask 라우트. ROS 를 직접 import 하지 않고 node 객체의 메서드만 호출한다.

node 는 web_server.ConsoleNode (테스트에서는 가짜 객체)이다.
"""

import math
import os

from flask import Flask, abort, jsonify, request, send_from_directory

from .ir_record import IrRecordError
from .map_store import MapNameError

# web/ 폴더에서 이 파일들만 서빙 (원본은 스크립트 폴더 전체를 static 으로 노출했음)
STATIC_FILES = {'index.html', 'app.js', 'style.css', 'ir.html', 'ir.js'}

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


def create_app(node, web_dir):
    app = Flask(__name__, static_folder=None)
    app.config['MAX_CONTENT_LENGTH'] = 16 * 1024

    @app.errorhandler(BadRequest)
    def _bad_request(e):
        return jsonify({'success': False, 'msg': str(e)}), 400

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
