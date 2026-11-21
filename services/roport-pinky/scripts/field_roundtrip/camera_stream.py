#!/usr/bin/env python3
"""Publish Pinky's Picamera2 frames as compressed ROS 2 images."""

import time

import cv2
import rclpy
from libcamera import Transform
from picamera2 import Picamera2
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage


class PinkyCameraStream(Node):
    def __init__(self):
        super().__init__('pinky_camera_stream')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.publisher = self.create_publisher(
            CompressedImage, '/camera/image_raw/compressed', qos
        )
        self.camera = Picamera2()
        config = self.camera.create_video_configuration(
            main={'size': (640, 480), 'format': 'RGB888'},
            controls={'FrameRate': 10.0},
            transform=Transform(hflip=1, vflip=1),
            buffer_count=2,
        )
        self.camera.configure(config)
        self.camera.start()
        time.sleep(1.0)
        self.timer = self.create_timer(0.1, self.publish_frame)
        self.get_logger().info('Publishing /camera/image_raw/compressed at 640x480, 10 FPS')

    def publish_frame(self):
        # Picamera2's RGB888 buffer is already in the byte order OpenCV expects
        # on this Pinky image. Converting RGB->BGR here swaps blue tape to orange.
        frame_bgr = self.camera.capture_array('main')
        ok, encoded = cv2.imencode('.jpg', frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 72])
        if not ok:
            self.get_logger().warning('JPEG encoding failed')
            return
        message = CompressedImage()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'camera_link'
        message.format = 'jpeg'
        message.data = encoded.tobytes()
        self.publisher.publish(message)

    def destroy_node(self):
        try:
            self.camera.stop()
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = PinkyCameraStream()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
