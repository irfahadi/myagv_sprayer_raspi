#!/usr/bin/env python3
"""Publish the saved nursery map as a latched ``nav_msgs/OccupancyGrid``.

Nav2 has been removed from this workspace -- the Layer-2 comparison needs A*,
D* Lite and the RL policy to run under identical rules, which is easier to
guarantee with our own controller than inside Nav2's plugin machinery. What was
still worth keeping from that stack is the map format, so this reads the same
``.pgm`` + ``.yaml`` pair ``nav2_map_server`` would, with no Nav2 dependency.

Only the map-based algorithms (A*, D* Lite) subscribe. VFH-QL never does; that
independence from a global map is the property `docs/experiment_protocol.md`
(Jetson repo) builds its whole comparison around.
"""
from __future__ import annotations

import os

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)


def read_pgm(path: str) -> np.ndarray:
    """Binary (P5) or ASCII (P2) PGM, as written by map_saver."""
    with open(path, 'rb') as f:
        data = f.read()
    tokens = []
    i = 0
    while len(tokens) < 4:
        while i < len(data) and data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            while i < len(data) and data[i] != 0x0A:
                i += 1
            continue
        j = i
        while j < len(data) and not data[j:j + 1].isspace():
            j += 1
        tokens.append(data[i:j])
        i = j
    magic, w, h, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    i += 1
    if magic == b'P5':
        dtype = np.uint8 if maxval < 256 else '>u2'
        img = np.frombuffer(data, dtype=dtype, count=w * h, offset=i)
    elif magic == b'P2':
        img = np.array(data[i:].split()[:w * h], dtype=np.int32)
    else:
        raise ValueError(f'{path}: unsupported PGM magic {magic!r}')
    return np.asarray(img, dtype=np.float64).reshape(h, w)


class MapServerNode(Node):

    def __init__(self):
        super().__init__('map_server_node')
        default = os.path.join(get_package_share_directory('myagv_sprayer_navigation'),
                               'maps', 'nursery_room.yaml')
        self.declare_parameter('map_yaml', default)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_period_s', 0.0)   # 0 = latched only

        path = self.get_parameter('map_yaml').value
        with open(path) as f:
            meta = yaml.safe_load(f)
        img_path = meta['image']
        if not os.path.isabs(img_path):
            img_path = os.path.join(os.path.dirname(os.path.abspath(path)), img_path)

        img = read_pgm(img_path)
        maxv = img.max() if img.max() > 0 else 255.0
        p = img / maxv
        if int(meta.get('negate', 0)):
            p = 1.0 - p
        occ = 1.0 - p                                  # dark pixels are occupied
        occupied = occ >= float(meta.get('occupied_thresh', 0.65))
        free = occ <= float(meta.get('free_thresh', 0.196))
        values = np.full(occ.shape, -1, dtype=np.int8)
        values[free] = 0
        values[occupied] = 100
        values = np.flipud(values)                     # PGM row 0 is the TOP row

        grid = OccupancyGrid()
        grid.header.frame_id = self.get_parameter('frame_id').value
        grid.info.resolution = float(meta['resolution'])
        grid.info.width = int(values.shape[1])
        grid.info.height = int(values.shape[0])
        grid.info.origin.position.x = float(meta['origin'][0])
        grid.info.origin.position.y = float(meta['origin'][1])
        grid.info.origin.orientation.w = 1.0
        grid.data = values.ravel().tolist()
        self.grid = grid

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(OccupancyGrid, '/map', qos)
        self._publish()
        period = float(self.get_parameter('publish_period_s').value)
        if period > 0:
            self.create_timer(period, self._publish)
        self.get_logger().info(
            f'{os.path.basename(path)}: {grid.info.width}x{grid.info.height} @ '
            f'{grid.info.resolution} m, origin '
            f'({grid.info.origin.position.x:.2f}, {grid.info.origin.position.y:.2f})')

    def _publish(self):
        self.grid.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.grid)


def main(args=None):
    rclpy.init(args=args)
    node = MapServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
