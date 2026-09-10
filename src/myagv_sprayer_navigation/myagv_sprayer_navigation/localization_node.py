#!/usr/bin/env python3
"""Publish ``map -> odom``, optionally with injected drift (Scenario B).

Two modes:

``identity``
    ``odom`` is already expressed in the map frame -- true in Gazebo, where the
    drive plugin publishes world-referenced odometry. This is the clean baseline.

``drift``
    The same transform, corrupted by a random walk plus a fixed initial offset.
    This is how Scenario B ("localization noise") is produced: nothing about the
    sensors changes, only the robot's belief about where it is.

That asymmetry is the experiment. A* and D* Lite consume this transform to decide
where they are on the map, so their paths degrade with it. The RL policy never
looks it up: its goal signal is the ArUco bearing, and its obstacle signal is the
raw scan, both in the robot frame.

On real hardware, run ``slam_toolbox`` in localization mode instead and leave
this node out (``mode:=off``) -- except when you deliberately want to inject
drift on top of it for an experiment.
"""
from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster


class LocalizationNode(Node):

    def __init__(self):
        super().__init__('localization_node')
        self.declare_parameter('mode', 'identity')       # identity | drift | off
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('drift_sigma_xy', 0.010)  # per second, random walk
        self.declare_parameter('drift_sigma_theta', 0.010)
        self.declare_parameter('initial_offset_m', 0.0)
        self.declare_parameter('initial_offset_theta', 0.0)
        self.declare_parameter('seed', 0)

        self.mode = str(self.get_parameter('mode').value)
        self.rng = np.random.default_rng(int(self.get_parameter('seed').value))
        self.offset = np.zeros(3)
        self._seed_offset()

        self.br = TransformBroadcaster(self)
        self.create_service(Trigger, '/localization/reset', self._reset)
        rate = float(self.get_parameter('rate_hz').value)
        self.dt = 1.0 / rate
        if self.mode != 'off':
            self.create_timer(self.dt, self._tick)
        self.get_logger().info(f'localization mode={self.mode}')

    def _seed_offset(self):
        d = float(self.get_parameter('initial_offset_m').value)
        if d > 0:
            a = self.rng.uniform(-math.pi, math.pi)
            self.offset[0] = d * math.cos(a)
            self.offset[1] = d * math.sin(a)
        self.offset[2] = float(self.get_parameter('initial_offset_theta').value)

    def _reset(self, req, resp):
        self.offset[:] = 0.0
        self._seed_offset()
        resp.success = True
        resp.message = 'localization offset reset'
        return resp

    def _tick(self):
        if self.mode == 'drift':
            sx = float(self.get_parameter('drift_sigma_xy').value) * math.sqrt(self.dt)
            st = float(self.get_parameter('drift_sigma_theta').value) * math.sqrt(self.dt)
            self.offset[0] += self.rng.normal(0.0, sx)
            self.offset[1] += self.rng.normal(0.0, sx)
            self.offset[2] += self.rng.normal(0.0, st)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.get_parameter('map_frame').value
        t.child_frame_id = self.get_parameter('odom_frame').value
        t.transform.translation.x = float(self.offset[0])
        t.transform.translation.y = float(self.offset[1])
        t.transform.rotation.z = math.sin(self.offset[2] / 2.0)
        t.transform.rotation.w = math.cos(self.offset[2] / 2.0)
        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
