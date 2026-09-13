#!/usr/bin/env python3
"""Spawn and clear Scenario 2's unmapped obstacles inside a running Gazebo.

Keeping the scenarios in one world file plus a spawner (rather than one world
file per scenario) matters for the protocol: every algorithm must face a
*byte-identical* world in a given trial, and the easiest way to guarantee that
is to place the obstacles from a seed rather than by hand.

    ros2 run myagv_sprayer_gazebo scenario_spawner_node --ros-args -p scenario:=2_unmapped_persistent
    ros2 service call /scenario/apply sprayer... (no custom srv needed)

Services
    /scenario/apply   std_srvs/Trigger   place the obstacles for ``scenario``+``trial``
    /scenario/clear   std_srvs/Trigger   delete everything this node spawned

For ``2_unmapped_transient`` the obstacles are additionally removed on their
own after ``transient_after_s`` -- the Gazebo-side counterpart of
``sprayer_nav/sim2d/room_env.py``'s ``_maybe_clear_transient_obstacles``, at the
same 120-step / 0.2 s-per-step point in wall-clock time (24 s).

The obstacle positions mirror ``sprayer_nav/sim2d/room_env.py`` so a result found
in the fast 2D benchmark can be reproduced here without re-deriving the layout.
See ``docs/experiment_protocol.md`` in the Jetson repo for the current scenario
matrix (1_ideal, 2_unmapped_persistent, 2_unmapped_transient).
"""
from __future__ import annotations

import os

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Pose
from rclpy.node import Node
from std_srvs.srv import Trigger

# (x, y) candidates for a bucket or a hose coil left in an aisle.
UNMAPPED_SPOTS = [
    (-1.3, 1.5, 'bucket'), (1.3, 1.5, 'hose_coil'), (0.0, 0.9, 'bucket'),
    (-1.6, -1.0, 'hose_coil'), (1.6, -1.0, 'bucket'), (0.0, -0.6, 'hose_coil'),
    (-2.2, 0.4, 'bucket'), (2.2, 0.4, 'hose_coil'),
]

SCENARIO_OBSTACLES = {
    '1_ideal': 0,
    '2_unmapped_persistent': 2,
    '2_unmapped_transient': 2,
}


def episode_seed(scenario: str, trial: int) -> int:
    """Same FNV-1a hash the offline benchmark uses, so trials line up."""
    h = 1469598103934665603
    for ch in f'{scenario}#{trial}':
        h = ((h ^ ord(ch)) * 1099511628211) & 0xFFFFFFFFFFFFFFFF
    return h % (2 ** 31)


class ScenarioSpawnerNode(Node):

    def __init__(self):
        super().__init__('scenario_spawner_node')
        self.declare_parameter('scenario', '1_ideal')
        self.declare_parameter('trial', 0)
        self.declare_parameter('apply_on_start', True)
        self.declare_parameter('transient_after_s', 24.0)

        self.models_dir = os.path.join(
            get_package_share_directory('myagv_sprayer_gazebo'), 'models')
        self.spawned = []
        self._transient_timer = None

        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_cli = self.create_client(DeleteEntity, '/delete_entity')
        self.create_service(Trigger, '/scenario/apply', self._apply_srv)
        self.create_service(Trigger, '/scenario/clear', self._clear_srv)

        if self.get_parameter('apply_on_start').value:
            self.create_timer(2.0, self._apply_once)
        self._applied = False

    # ------------------------------------------------------------------ helpers
    def _sdf(self, model: str) -> str:
        with open(os.path.join(self.models_dir, model, 'model.sdf')) as f:
            return f.read()

    def _wait(self, cli, name):
        if not cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(f'{name} unavailable -- is Gazebo running?')
            return False
        return True

    def _spawn(self, model: str, name: str, x: float, y: float, yaw: float = 0.0):
        if not self._wait(self.spawn_cli, '/spawn_entity'):
            return False
        req = SpawnEntity.Request()
        req.name = name
        req.xml = self._sdf(model)
        p = Pose()
        p.position.x = float(x)
        p.position.y = float(y)
        p.orientation.z = float(np.sin(yaw / 2.0))
        p.orientation.w = float(np.cos(yaw / 2.0))
        req.initial_pose = p
        fut = self.spawn_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        ok = fut.result() is not None and fut.result().success
        if ok:
            self.spawned.append(name)
            self.get_logger().info(f'spawned {name} ({model}) at ({x:.2f}, {y:.2f})')
        else:
            self.get_logger().error(f'failed to spawn {name}')
        return ok

    def _clear(self):
        if not self.spawned:
            return 0
        if not self._wait(self.delete_cli, '/delete_entity'):
            return 0
        n = 0
        for name in list(self.spawned):
            req = DeleteEntity.Request()
            req.name = name
            fut = self.delete_cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
            self.spawned.remove(name)
            n += 1
        self.get_logger().info(f'cleared {n} obstacle(s)')
        return n

    # ------------------------------------------------------------------ scenarios
    def apply(self):
        scenario = str(self.get_parameter('scenario').value)
        trial = int(self.get_parameter('trial').value)
        self._clear()
        if self._transient_timer is not None:
            self._transient_timer.cancel()
            self._transient_timer = None
        rng = np.random.default_rng(episode_seed(scenario, trial))

        n = SCENARIO_OBSTACLES.get(scenario, 0)
        spots = list(UNMAPPED_SPOTS)
        rng.shuffle(spots)
        for i, (x, y, model) in enumerate(spots[:n]):
            self._spawn(model, f'scenario_obstacle_{i}', x, y)

        if scenario == '2_unmapped_transient' and self.spawned:
            after_s = float(self.get_parameter('transient_after_s').value)
            self._transient_timer = self.create_timer(after_s, self._clear_transient_once)

        self.get_logger().info(
            f'scenario {scenario} trial {trial}: {len(self.spawned)} obstacle(s). '
            'Remember: the saved map does NOT contain them -- that is the point.')
        return len(self.spawned)

    def _clear_transient_once(self):
        """Scenario 2's transient variant: the hose gets picked up mid-episode --
        mirrors ``RoomEnv._maybe_clear_transient_obstacles`` in the 2D benchmark."""
        if self._transient_timer is not None:
            self._transient_timer.cancel()
            self._transient_timer = None
        n = self._clear()
        self.get_logger().info(f'transient obstacle window elapsed: cleared {n}')

    def _apply_once(self):
        if self._applied:
            return
        self._applied = True
        self.apply()

    def _apply_srv(self, req, resp):
        n = self.apply()
        resp.success = True
        resp.message = f'{n} obstacle(s) placed'
        return resp

    def _clear_srv(self, req, resp):
        n = self._clear()
        resp.success = True
        resp.message = f'{n} obstacle(s) removed'
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = ScenarioSpawnerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
