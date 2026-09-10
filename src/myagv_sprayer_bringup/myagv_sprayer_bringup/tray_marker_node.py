#!/usr/bin/env python3
"""Publish the four seedling trays and their parking poses as RViz markers,
and broadcast a static TF ``map -> <tray>`` for each tray centre.

Reads myagv_sprayer_navigation/config/tray_waypoints.yaml.
"""
import math
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


def yaw_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class TrayMarkerNode(Node):

    def __init__(self):
        super().__init__('tray_marker_node')
        default = os.path.join(get_package_share_directory('myagv_sprayer_navigation'),
                               'config', 'tray_waypoints.yaml')
        self.declare_parameter('waypoints_file', default)
        path = self.get_parameter('waypoints_file').value
        with open(path) as f:
            cfg = yaml.safe_load(f)['tray_waypoints']['ros__parameters']
        self.frame = cfg.get('frame_id', 'map')
        self.trays = cfg['tray_names']
        self.cfg = cfg

        self.pub = self.create_publisher(MarkerArray, '/sprayer/markers', 1)
        self.tf = StaticTransformBroadcaster(self)
        self._broadcast_tf()
        self.create_timer(1.0, self._publish)

    def _broadcast_tf(self):
        tfs = []
        for name in self.trays:
            cx, cy = self.cfg[f'{name}_centre']
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.frame
            t.child_frame_id = name
            t.transform.translation.x = float(cx)
            t.transform.translation.y = float(cy)
            t.transform.translation.z = 0.25
            t.transform.rotation.w = 1.0
            tfs.append(t)
        self.tf.sendTransform(tfs)

    def _publish(self):
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()
        for i, name in enumerate(self.trays):
            cx, cy = self.cfg[f'{name}_centre']
            px, py, pyaw = self.cfg[name]

            tray = Marker()
            tray.header.frame_id = self.frame
            tray.header.stamp = now
            tray.ns = 'trays'
            tray.id = i
            tray.type = Marker.CUBE
            tray.action = Marker.ADD
            tray.pose.position.x = float(cx)
            tray.pose.position.y = float(cy)
            tray.pose.position.z = 0.125
            tray.pose.orientation.w = 1.0
            tray.scale.x, tray.scale.y, tray.scale.z = 1.2, 0.7, 0.25
            tray.color.r, tray.color.g, tray.color.b, tray.color.a = 0.2, 0.7, 0.2, 0.5
            arr.markers.append(tray)

            park = Marker()
            park.header = tray.header
            park.ns = 'park'
            park.id = i
            park.type = Marker.ARROW
            park.action = Marker.ADD
            park.pose.position.x = float(px)
            park.pose.position.y = float(py)
            park.pose.position.z = 0.05
            qx, qy, qz, qw = yaw_quat(float(pyaw))
            park.pose.orientation.x, park.pose.orientation.y = qx, qy
            park.pose.orientation.z, park.pose.orientation.w = qz, qw
            park.scale.x, park.scale.y, park.scale.z = 0.4, 0.06, 0.06
            park.color.r, park.color.g, park.color.b, park.color.a = 0.1, 0.6, 1.0, 0.9
            arr.markers.append(park)

            label = Marker()
            label.header = tray.header
            label.ns = 'labels'
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(cx)
            label.pose.position.y = float(cy)
            label.pose.position.z = 0.6
            label.pose.orientation.w = 1.0
            label.scale.z = 0.2
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = name
            arr.markers.append(label)
        self.pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = TrayMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
