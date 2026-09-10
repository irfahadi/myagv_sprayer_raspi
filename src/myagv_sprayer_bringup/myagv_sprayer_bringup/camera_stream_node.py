#!/usr/bin/env python3
"""Publish the myAGV front camera as a LAN-friendly JPEG stream.

Two sources are supported (parameter ``source``):

* ``v4l2``  – real robot: grab frames from ``/dev/videoN`` with OpenCV
              (the built-in myAGV USB camera is wired to the Raspberry Pi).
* ``topic`` – simulation: subscribe to ``sensor_msgs/Image`` published by the
              Gazebo camera plugin and re-encode it.

In both cases the node publishes:

* ``/camera/image_raw/compressed``  (sensor_msgs/CompressedImage, JPEG)
* ``/camera/camera_info``           (sensor_msgs/CameraInfo)          [v4l2 only]

The Jetson Nano subscribes to the compressed topic over Ethernet, which keeps
DDS traffic at ~1-2 MB/s instead of ~14 MB/s for raw 640x480 @ 15 Hz.
"""
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


class CameraStreamNode(Node):

    def __init__(self):
        super().__init__('camera_stream_node')

        self.declare_parameter('source', 'v4l2')          # v4l2 | topic
        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 10.0)
        self.declare_parameter('jpeg_quality', 75)
        self.declare_parameter('frame_id', 'camera_optical_frame')
        # pinhole intrinsics for the stock myAGV camera at 640x480 (calibrate!)
        self.declare_parameter('fx', 520.0)
        self.declare_parameter('fy', 520.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)

        self.source = self.get_parameter('source').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.quality = int(self.get_parameter('jpeg_quality').value)
        self.frame_id = self.get_parameter('frame_id').value

        sensor_qos = QoSProfile(depth=1,
                                reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST)

        self.pub_jpeg = self.create_publisher(
            CompressedImage, '/camera/image_raw/compressed', sensor_qos)
        self.pub_info = self.create_publisher(
            CameraInfo, '/camera/camera_info', sensor_qos)

        self.bridge = CvBridge()
        self._last_pub = 0.0
        self._min_period = 1.0 / max(self.fps, 0.1)

        if self.source == 'v4l2':
            self.cap = cv2.VideoCapture(self.get_parameter('device').value)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS, int(self.fps))
            if not self.cap.isOpened():
                self.get_logger().error(
                    f"Cannot open {self.get_parameter('device').value}")
            self.create_timer(self._min_period, self._grab)
            self.get_logger().info(
                f'Streaming {self.get_parameter("device").value} '
                f'{self.width}x{self.height} @ {self.fps} Hz (JPEG q={self.quality})')
        else:
            self.create_subscription(
                Image, self.get_parameter('input_topic').value,
                self._on_image, sensor_qos)
            self.get_logger().info(
                f'Re-encoding {self.get_parameter("input_topic").value} '
                f'-> /camera/image_raw/compressed @ {self.fps} Hz')

    # ---------------------------------------------------------------- v4l2
    def _grab(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn('camera read failed', throttle_duration_sec=5.0)
            return
        stamp = self.get_clock().now().to_msg()
        self._publish(frame, stamp)
        self.pub_info.publish(self._camera_info(stamp))

    # ---------------------------------------------------------------- topic
    def _on_image(self, msg: Image):
        now = time.monotonic()
        if now - self._last_pub < self._min_period:
            return
        self._last_pub = now
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self._publish(frame, msg.header.stamp)

    # ---------------------------------------------------------------- common
    def _publish(self, frame: np.ndarray, stamp):
        if frame.shape[1] != self.width or frame.shape[0] != self.height:
            frame = cv2.resize(frame, (self.width, self.height))
        ok, buf = cv2.imencode('.jpg', frame,
                               [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.format = 'jpeg'
        msg.data = buf.tobytes()
        self.pub_jpeg.publish(msg)

    def _camera_info(self, stamp) -> CameraInfo:
        fx = self.get_parameter('fx').value
        fy = self.get_parameter('fy').value
        cx = self.get_parameter('cx').value
        cy = self.get_parameter('cy').value
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.frame_id
        info.width = self.width
        info.height = self.height
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info


def main(args=None):
    rclpy.init(args=args)
    node = CameraStreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
