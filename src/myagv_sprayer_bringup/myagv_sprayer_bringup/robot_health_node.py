#!/usr/bin/env python3
"""Raspberry Pi health monitor: CPU temperature, load, throttling flags and
the myAGV battery voltage (if the driver publishes it).

Publishes ``/myagv/health`` (diagnostic_msgs/DiagnosticArray) at 1 Hz so the
Jetson mission node can pause spraying when the base is unhealthy.
"""
import os
import subprocess

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from std_msgs.msg import Float32


def read_cpu_temp():
    try:
        with open('/sys/class/thermal/thermal_zone0/temp') as f:
            return int(f.read().strip()) / 1000.0
    except OSError:
        return float('nan')


def read_throttled():
    """vcgencmd get_throttled -> hex flags (0x0 = ok). Returns None if unavailable."""
    try:
        out = subprocess.run(['vcgencmd', 'get_throttled'], capture_output=True,
                             text=True, timeout=1.0).stdout
        return int(out.strip().split('=')[1], 16)
    except Exception:
        return None


class RobotHealthNode(Node):

    def __init__(self):
        super().__init__('robot_health_node')
        self.declare_parameter('battery_topic', '/myagv/battery_voltage')
        self.declare_parameter('battery_low_v', 10.5)
        self.declare_parameter('cpu_temp_warn_c', 75.0)

        self.battery_v = float('nan')
        self.create_subscription(Float32, self.get_parameter('battery_topic').value,
                                 self._on_batt, 10)
        self.pub = self.create_publisher(DiagnosticArray, '/myagv/health', 10)
        self.create_timer(1.0, self._tick)

    def _on_batt(self, msg: Float32):
        self.battery_v = msg.data

    def _tick(self):
        temp = read_cpu_temp()
        load1 = os.getloadavg()[0]
        throttled = read_throttled()

        st = DiagnosticStatus()
        st.name = 'myagv_raspi'
        st.hardware_id = 'raspberry-pi-4b'
        st.level = DiagnosticStatus.OK
        st.message = 'ok'

        if temp > self.get_parameter('cpu_temp_warn_c').value:
            st.level = DiagnosticStatus.WARN
            st.message = 'cpu hot'
        if throttled not in (None, 0):
            st.level = DiagnosticStatus.WARN
            st.message = f'throttled 0x{throttled:x}'
        if self.battery_v == self.battery_v and \
                self.battery_v < self.get_parameter('battery_low_v').value:
            st.level = DiagnosticStatus.ERROR
            st.message = 'battery low'

        st.values = [
            KeyValue(key='cpu_temp_c', value=f'{temp:.1f}'),
            KeyValue(key='load1', value=f'{load1:.2f}'),
            KeyValue(key='throttled', value='n/a' if throttled is None else hex(throttled)),
            KeyValue(key='battery_v', value=f'{self.battery_v:.2f}'),
        ]
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [st]
        self.pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = RobotHealthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
