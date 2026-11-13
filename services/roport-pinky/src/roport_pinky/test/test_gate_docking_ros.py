"""Synthetic ROS camera + action peer; proves wiring, never physical docking."""
from uuid import uuid4

import cv2
import numpy as np
import pytest
import yaml
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, CompressedImage
from tf2_ros import StaticTransformBroadcaster

from ros_trial_fixture import RosTrial
from test_gate_docking import CONFIG
from test_trial_driver import PROFILE


@pytest.mark.parametrize('camera_available', [True, False])
def test_gate_camera_stage_preserves_original_control_completion(tmp_path, monkeypatch, camera_available):
    path = tmp_path/'gate.yaml'
    path.write_text(yaml.safe_dump(CONFIG))
    monkeypatch.setenv('ROPORT_GATE_DOCK_CONFIG', str(path))
    trial = RosTrial()
    try:
        trial.driver.mode = 'nav2'
        trial.pose = dict(x=1.9, y=0., yaw=0.)
        trial.wait(lambda: abs(trial.driver.nav.snapshot()['pose']['x']-1.9) < .01)
        broadcaster = StaticTransformBroadcaster(trial.node)
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = 'base_footprint', 'gate_camera_optical'
        tf.header.stamp = trial.node.get_clock().now().to_msg()
        tf.transform.translation.x, tf.transform.translation.z = .03, .1
        q = tf.transform.rotation
        q.x,q.y,q.z,q.w = -.5,.5,-.5,.5
        broadcaster.sendTransform(tf)
        info_pub = trial.node.create_publisher(CameraInfo, 'camera/camera_info', 1)
        image_pub = trial.node.create_publisher(CompressedImage, 'camera/image_raw/compressed', 1)
        def camera():
            if not camera_available:
                return
            info = CameraInfo()
            info.header.frame_id = 'gate_camera_optical'
            info.header.stamp = trial.node.get_clock().now().to_msg()
            info.width,info.height = 480,360
            info.distortion_model,info.d = 'plumb_bob',[0.]*5
            info.k = [200.,0.,240.,0.,200.,180.,0.,0.,1.]
            info_pub.publish(info)
            distance = 2.4-(trial.pose['x']+.03)
            size = round(200*.1/distance)
            image = np.full((360,480),255,np.uint8)
            if 8 <= size < 350:
                marker = cv2.aruco.drawMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),9,size)
                top,left = (360-size)//2,(480-size)//2
                image[top:top+size,left:left+size] = marker
            _, encoded = cv2.imencode('.jpg',image)
            msg = CompressedImage()
            msg.header, msg.format, msg.data = info.header, 'jpeg', encoded.tobytes()
            image_pub.publish(msg)
        timer = trial.node.create_timer(.18,camera)
        cmd = {'type':'navigate.requested','command_id':str(uuid4()), 'execution_id':str(uuid4()),
               'payload':{**PROFILE,'frame_id':'map','goal_id':'G2','goal_version':'v-test',
                          'pose':dict(x=2.29,y=0.,yaw=0.),
                          'position_tolerance_m':.03,'yaw_tolerance_rad':.1}}
        result = trial.run(cmd)[-1]
        if camera_available:
            assert result[1] == 'completed', result
            assert result[2]['pose']['x'] == pytest.approx(2.29, abs=.015)
            assert trial.driver.nav.dock_phase is None
        else:
            assert result[1:] == ('failed',{'error_code':'gate_dock_marker_missing'})
            assert trial.pose['x'] == pytest.approx(2.09)  # approach only; no guessed final goal
        assert trial.driver.nav.snapshot()['stopped']
        timer.cancel()
    finally:
        trial.close()
