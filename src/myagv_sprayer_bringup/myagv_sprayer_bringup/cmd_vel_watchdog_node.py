#!/usr/bin/env python3
"""Safety gate between the Jetson and the wheels.

Every navigation decision now happens on the Jetson Nano, which means the
command path crosses an Ethernet link. This node is the only publisher on
``/cmd_vel``: it relays ``/cmd_vel_nav`` and publishes a zero twist the moment
the stream goes quiet, so a dropped link, a crashed node or a paused debugger
stops the robot instead of letting it coast at the last commanded speed.

It also clamps speed and applies acceleration limits, which protects the
mecanum drivetrain from the step commands a discrete-action policy produces
(the policy switches between 45-degree directions instantly; the wheels should
not be asked to).

    /cmd_vel_nav ---.
                    |-> [watchdog: timeout, clamp, ramp, e-stop] -> /cmd_vel
    /cmd_vel_manual-'
"""
from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String


def clamp(v, lim):
    return max(-lim, min(lim, v))


def ramp(cur, target, max_delta):
    if target > cur + max_delta:
        return cur + max_delta
    if target < cur - max_delta:
        return cur - max_delta
    return target


class CmdVelWatchdogNode(Node):

    def __init__(self):
        super().__init__('cmd_vel_watchdog_node')
        self.declare_parameter('input_topic', '/cmd_vel_nav')
        self.declare_parameter('manual_topic', '/cmd_vel_manual')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('timeout_s', 0.5)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('max_linear', 0.35)
        self.declare_parameter('max_lateral', 0.30)
        self.declare_parameter('max_angular', 1.5)
        self.declare_parameter('accel_linear', 0.8)      # m/s^2
        self.declare_parameter('accel_angular', 3.0)     # rad/s^2
        self.declare_parameter('manual_priority_s', 1.0)

        self.timeout = float(self.get_parameter('timeout_s').value)
        self.rate = float(self.get_parameter('rate_hz').value)
        self.dt = 1.0 / self.rate

        self.cmd = Twist()
        self.cmd_stamp = 0.0
        self.manual = Twist()
        self.manual_stamp = 0.0
        self.estop = False
        self.out = Twist()
        self.was_active = False

        self.create_subscription(Twist, self.get_parameter('input_topic').value,
                                 self._on_cmd, 10)
        self.create_subscription(Twist, self.get_parameter('manual_topic').value,
                                 self._on_manual, 10)
        self.create_subscription(Bool, '/sprayer/estop', self._on_estop, 10)
        self.pub = self.create_publisher(
            Twist, self.get_parameter('output_topic').value, 10)
        self.pub_reason = self.create_publisher(String, '/cmd_vel_watchdog/state', 10)
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f'watchdog: {self.get_parameter("input_topic").value} -> '
            f'{self.get_parameter("output_topic").value}, '
            f'timeout {self.timeout:.2f} s')

    def _on_cmd(self, msg: Twist):
        self.cmd = msg
        self.cmd_stamp = time.monotonic()

    def _on_manual(self, msg: Twist):
        self.manual = msg
        self.manual_stamp = time.monotonic()

    def _on_estop(self, msg: Bool):
        if msg.data and not self.estop:
            self.get_logger().warn('E-STOP: wheels held at zero')
        self.estop = bool(msg.data)

    def _tick(self):
        now = time.monotonic()
        reason = 'active'
        target = Twist()

        if self.estop:
            reason = 'estop'
        elif now - self.manual_stamp < float(self.get_parameter('manual_priority_s').value):
            target = self.manual                     # a human at the joystick wins
            reason = 'manual'
        elif now - self.cmd_stamp < self.timeout:
            target = self.cmd
        else:
            reason = 'timeout'
            if self.was_active:
                self.get_logger().warn(
                    'no command for %.2f s (Jetson link down?); stopping'
                    % (now - self.cmd_stamp))

        lin = float(self.get_parameter('max_linear').value)
        lat = float(self.get_parameter('max_lateral').value)
        ang = float(self.get_parameter('max_angular').value)
        dv = float(self.get_parameter('accel_linear').value) * self.dt
        dw = float(self.get_parameter('accel_angular').value) * self.dt

        self.out.linear.x = ramp(self.out.linear.x, clamp(target.linear.x, lin), dv)
        self.out.linear.y = ramp(self.out.linear.y, clamp(target.linear.y, lat), dv)
        self.out.angular.z = ramp(self.out.angular.z, clamp(target.angular.z, ang), dw)
        self.pub.publish(self.out)
        self.pub_reason.publish(String(data=reason))
        self.was_active = reason == 'active'


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelWatchdogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
