#!/usr/bin/env python3
"""Fail if the three repos disagree about the geometry they all encode.

The same room, robot and tray numbers are written down in four places, and
nothing but this script notices when one drifts:

    myagv_sprayer_raspi/.../worlds/nursery_room.world        the Gazebo world
    myagv_sprayer_raspi/.../urdf/*.xacro                     the robot
    myagv_sprayer_jetson/.../sim2d/layout.py                 the 2D simulator
    myagv_sprayer_jetson/.../config/*.yaml                   the live stack

A mismatch is not cosmetic: the Q-tables in qtables/ were trained against
layout.py, so a world that no longer matches it is asking the policy about
states it never saw. ``sim2d/layout.py``'s own module docstring promises this
script exists -- for a long time it did not.

    python3 tools/check_layout.py            # exit 0 = agreed, 1 = drifted
    python3 tools/check_layout.py -v         # also print the checks that passed

Reads only; it never edits anything. The xacro files are parsed for literal
<xacro:property> values rather than processed, which is enough because every
number this compares is a plain literal.
"""
from __future__ import annotations

import argparse
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
GAZEBO = os.path.dirname(HERE)                        # .../myagv_sprayer_gazebo
RASPI_SRC = os.path.dirname(GAZEBO)                   # .../myagv_sprayer_raspi/src
WS_SRC = os.path.dirname(os.path.dirname(RASPI_SRC))  # .../ros2_ws/src

WORLD = os.path.join(GAZEBO, 'worlds', 'nursery_room.world')
TRAY_SDF = os.path.join(GAZEBO, 'models', 'seedling_tray', 'model.sdf')
MARKER_GEN = os.path.join(HERE, 'generate_aruco_markers.py')
DESC = os.path.join(RASPI_SRC, 'myagv_sprayer_description', 'urdf')
RASPI_WP = os.path.join(RASPI_SRC, 'myagv_sprayer_navigation', 'config',
                        'tray_waypoints.yaml')

TOL = 1e-3          # metres / radians. Every number here is a literal, so this
                    # only absorbs float formatting, not real disagreement.

failures: list[str] = []
passes: list[str] = []


def check(name: str, got, want, tol: float = TOL, unit: str = ''):
    """Record one comparison. Scalars compare numerically, anything else by ==."""
    if isinstance(got, (int, float)) and isinstance(want, (int, float)):
        ok = abs(float(got) - float(want)) <= tol
    else:
        ok = got == want
    line = f'{name}: {got}{unit} vs {want}{unit}'
    (passes if ok else failures).append(line)
    return ok


def read(path: str) -> str:
    with open(path) as f:
        return f.read()


# --------------------------------------------------------------- the parsers

def xacro_properties(*files) -> dict:
    """Literal <xacro:property name="x" value="1.0"/> pairs, merged in order."""
    out = {}
    for f in files:
        for m in re.finditer(r'<xacro:property\s+name="([\w_]+)"\s+value="([^"]+)"',
                             read(os.path.join(DESC, f))):
            out[m.group(1)] = m.group(2)
    return out


def world_includes() -> list:
    """(model_uri, instance_name, pose_list) for every <include> in the world."""
    out = []
    for m in re.finditer(r'<include>(.*?)</include>', read(WORLD), re.S):
        body = m.group(1)
        uri = re.search(r'model://([\w/]+)', body)
        name = re.search(r'<name>([^<]+)</name>', body)
        pose = re.search(r'<pose>([^<]+)</pose>', body)
        if not uri:
            continue
        out.append((uri.group(1),
                    name.group(1) if name else '',
                    [float(v) for v in pose.group(1).split()] if pose else None))
    return out


def load_module(path: str, name: str):
    import importlib.util as u
    spec = u.spec_from_file_location(name, path)
    mod = u.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def load_yaml(path: str) -> dict:
    import yaml
    with open(path) as f:
        return yaml.safe_load(f)


# ----------------------------------------------------------------- the checks

def check_room(L):
    """Inner room size, from the wall boxes in the world.

    The north/south walls run the full outer width, so their <size> x is
    ROOM_X + 2*WALL_T; the east/west walls span the inner depth exactly.
    """
    sizes = [[float(v) for v in m.split()]
             for m in re.findall(r'<size>([-0-9.\s]+)</size>', read(WORLD))]
    walls = [s for s in sizes if len(s) == 3 and abs(s[2] - L.WALL_H) <= TOL]
    if not walls:
        failures.append(f'room: no wall box of height WALL_H={L.WALL_H} in the world')
        return
    ns = [s for s in walls if s[0] > s[1]]      # long in x
    ew = [s for s in walls if s[1] > s[0]]      # long in y
    if ns:
        check('ROOM_X (from N/S wall length - 2*WALL_T)',
              ns[0][0] - 2 * L.WALL_T, L.ROOM_X, unit=' m')
        check('WALL_T (N/S wall thickness)', ns[0][1], L.WALL_T, unit=' m')
    if ew:
        check('ROOM_Y (from E/W wall span)', ew[0][1], L.ROOM_Y, unit=' m')
        check('WALL_T (E/W wall thickness)', ew[0][0], L.WALL_T, unit=' m')


def check_tray_size(L):
    """TRAY_SX/TRAY_SY against the rim visual the markers are placed on."""
    m = re.search(r'<visual name="rim">.*?<size>([-0-9.\s]+)</size>',
                  read(TRAY_SDF), re.S)
    if not m:
        failures.append('tray_size: no <visual name="rim"> box in seedling_tray/model.sdf')
        return
    sx, sy, _sz = [float(v) for v in m.group(1).split()]
    check('TRAY_SX (seedling_tray rim)', sx, L.TRAY_SX, unit=' m')
    check('TRAY_SY (seedling_tray rim)', sy, L.TRAY_SY, unit=' m')


def check_tray_centres(L):
    """Every tray centre in layout.TRAYS must be spawned in the world."""
    spawned = sorted((round(p[0], 3), round(p[1], 3))
                     for uri, _n, p in world_includes()
                     if uri == 'seedling_tray' and p)
    declared = sorted((round(t['center'][0], 3), round(t['center'][1], 3))
                      for t in L.TRAYS)
    check('tray centres (world vs layout.TRAYS)', spawned, declared)


def check_markers(L):
    """The 12 ArUco stands, checked twice over.

    First against ``generate_aruco_markers.world_poses()``, which is what wrote
    the world. Then against ``layout.marker_poses()``, which the 2D simulator
    reads -- that one returns the marker ON THE RIM, while the world holds the
    STAND, set FACE_GAP clear of the rim along the outward normal. Comparing
    them without that offset would report drift that is not there.
    """
    gen = load_module(MARKER_GEN, 'gen')
    world = sorted((uri, round(p[0], 3), round(p[1], 3), round(p[5], 4))
                   for uri, _n, p in world_includes()
                   if uri.startswith('aruco_marker_') and p)

    expect = []
    for i, tray in enumerate(gen.TRAYS, start=1):
        for _face, x, y, yaw in gen.world_poses(tray):
            expect.append((f'aruco_marker_{i}', round(x, 3), round(y, 3), round(yaw, 4)))
    check('aruco stands (world vs generate_aruco_markers.world_poses)',
          world, sorted(expect))

    # layout.marker_poses() -> push each rim marker out by FACE_GAP, which is
    # where the world puts the stand.
    gap = float(getattr(gen, 'FACE_GAP', 0.0))
    from_layout = []
    for i, tray in enumerate(L.TRAYS, start=1):
        for (mx, my), (nx, ny) in L.marker_poses(tray):
            from_layout.append((f'aruco_marker_{i}',
                                round(mx + nx * gap, 3), round(my + ny * gap, 3)))
    check('aruco stands (world vs sim2d layout.marker_poses + FACE_GAP)',
          [(u, x, y) for u, x, y, _yaw in world], sorted(from_layout))


def check_camera(L, dock_cfg, vision_cfg):
    """The camera pose and intrinsics, against the URDF and the Gazebo sensor.

    Checked for BOTH detectors. There is one physical camera but two nodes carry
    its extrinsics -- aruco_detector_node in docking.yaml and tray_detector_node
    in tray_detector.yaml -- and a missed copy is how camera_z sat at 0.50 while
    the mast was 0.46. sprayer_docking's test_camera_geometry.py cross-checks the
    two configs against each other; this ties both to the robot they describe.
    """
    p = xacro_properties('common_properties.xacro', 'myagv_base.xacro',
                         'sprayer_payload.xacro')
    cams = [('docking.yaml', dock_cfg['aruco_detector_node']['ros__parameters']),
            ('tray_detector.yaml', vision_cfg['tray_detector_node']['ros__parameters'])]

    for key, prop in (('camera_x', 'camera_x'), ('camera_z', 'mast_top_z')):
        if prop not in p:
            failures.append(f'{key}: no <xacro:property name="{prop}"> found in '
                            f'the urdf -- this check did not run')
            continue
        for where, cam in cams:
            if key in cam:
                check(f'{key} ({where} vs urdf {prop})', cam[key],
                      float(p[prop]), unit=' m')

    joint = re.search(r'camera_joint.*?rpy="([^"]+)"',
                      read(os.path.join(DESC, 'myagv_base.xacro')), re.S)
    if joint:
        pitch = float(joint.group(1).split()[1])
        for where, cam in cams:
            if 'camera_pitch' in cam:
                check(f'camera_pitch ({where} vs urdf camera_joint rpy)',
                      cam['camera_pitch'], pitch, unit=' rad')

    sensors = read(os.path.join(DESC, 'sensors.gazebo.xacro'))
    fov = re.search(r'<horizontal_fov>([0-9.]+)</horizontal_fov>', sensors)
    width = re.search(r'<width>(\d+)</width>', sensors)
    if fov:
        check('CAMERA_HFOV (sim2d layout vs gazebo sensor)',
              L.CAMERA_HFOV, float(fov.group(1)), tol=2e-3, unit=' rad')
    if fov and width:
        # Fallbacks only: both nodes prefer /camera/camera_info. They still have
        # to be the RIGHT fallbacks -- fx = (width/2) / tan(hfov/2).
        w = float(width.group(1))
        fx = (w / 2.0) / math.tan(float(fov.group(1)) / 2.0)
        for where, cam in cams:
            if 'fx' in cam:
                check(f'{where} fallback fx (vs gazebo fov+width)',
                      cam['fx'], fx, tol=1.0)
            if 'cx' in cam:
                check(f'{where} fallback cx (vs gazebo width/2)',
                      cam['cx'], w / 2.0, tol=0.5)


def live_param(nav_cfg, node_src: str, key: str):
    """The value the live stack actually uses, and where it came from.

    A node-specific YAML entry wins over the node's own declare_parameter
    default, and some parameters are deliberately kept out of the YAML so a
    launch argument can override them -- so neither source alone is authoritative.
    """
    params = nav_cfg.get('nav_controller_node', {}).get('ros__parameters', {})
    if key in params:
        return params[key], 'nav_controller.yaml'
    m = re.search(r"declare_parameter\(\s*'" + re.escape(key)
                  + r"'\s*,\s*([-0-9.eE]+)\s*\)", read(node_src))
    if m:
        return float(m.group(1)), 'declare_parameter default'
    return None, 'nowhere'


def check_lidar(nav_cfg, node_src):
    """lidar_offset_x against where the URDF actually mounts the LiDAR."""
    base = read(os.path.join(DESC, 'myagv_base.xacro'))
    m = re.search(r'(?:laser|lidar)_joint.*?origin xyz="([^"]+)"', base, re.S)
    if not m:
        failures.append('lidar_offset_x: no laser/lidar joint origin in myagv_base.xacro')
        return
    got, src = live_param(nav_cfg, node_src, 'lidar_offset_x')
    if got is None:
        failures.append('lidar_offset_x: declared neither in the YAML nor in the node')
        return
    check(f'lidar_offset_x ({src} vs urdf)', got,
          float(m.group(1).split()[0]), unit=' m')


def check_waypoints(jetson_wp):
    """The two tray_waypoints.yaml copies must agree on every value.

    Comments may differ; the parsed parameter dicts may not. Only the RasPi copy
    is loaded by the navigation stack there, but a value that disagrees with the
    Jetson's means the two halves of the robot are aiming at different points.
    """
    a = load_yaml(RASPI_WP)['tray_waypoints']['ros__parameters']
    b = load_yaml(jetson_wp)['tray_waypoints']['ros__parameters']
    for key in sorted(set(a) | set(b)):
        if key not in a:
            failures.append(f'tray_waypoints[{key}]: missing from the RasPi copy')
        elif key not in b:
            failures.append(f'tray_waypoints[{key}]: missing from the Jetson copy')
        else:
            check(f'tray_waypoints[{key}]', a[key], b[key])


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--jetson', default=os.path.join(WS_SRC, 'myagv_sprayer_jetson'),
                    help='myagv_sprayer_jetson checkout (default: alongside this repo)')
    ap.add_argument('-v', '--verbose', action='store_true',
                    help='also list the checks that agreed')
    a = ap.parse_args()

    layout_py = os.path.join(a.jetson, 'src', 'sprayer_nav', 'sprayer_nav',
                             'sim2d', 'layout.py')
    if not os.path.exists(layout_py):
        print(f'cannot find sim2d/layout.py under {a.jetson}\n'
              f'pass --jetson <path to myagv_sprayer_jetson>', file=sys.stderr)
        return 2

    L = load_module(layout_py, 'layout')
    nav_cfg = load_yaml(os.path.join(a.jetson, 'src', 'sprayer_nav', 'config',
                                     'nav_controller.yaml'))
    dock_cfg = load_yaml(os.path.join(a.jetson, 'src', 'sprayer_docking', 'config',
                                      'docking.yaml'))
    vision_cfg = load_yaml(os.path.join(a.jetson, 'src', 'sprayer_vision', 'config',
                                        'tray_detector.yaml'))
    jetson_wp = os.path.join(a.jetson, 'src', 'sprayer_nav', 'config',
                             'tray_waypoints.yaml')
    node_src = os.path.join(a.jetson, 'src', 'sprayer_nav', 'sprayer_nav',
                            'nav_controller_node.py')

    check_room(L)
    check_tray_size(L)
    check_tray_centres(L)
    check_markers(L)
    check_camera(L, dock_cfg, vision_cfg)
    check_lidar(nav_cfg, node_src)
    check_waypoints(jetson_wp)

    if a.verbose:
        for line in passes:
            print(f'  ok    {line}')
    if failures:
        print(f'\nDRIFT -- {len(failures)} of {len(failures) + len(passes)} '
              f'checks disagree:\n', file=sys.stderr)
        for line in failures:
            print(f'  FAIL  {line}', file=sys.stderr)
        print('\nThe Q-tables in qtables/ were trained against sim2d/layout.py.\n'
              'Whichever side you change, say so in the commit message.',
              file=sys.stderr)
        return 1

    print(f'layout agreed across all three repos ({len(passes)} checks)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
