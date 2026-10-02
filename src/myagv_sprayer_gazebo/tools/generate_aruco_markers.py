#!/usr/bin/env python3
"""Generate the ArUco tray markers: Gazebo models plus printable PDFs/PNGs.

    python3 tools/generate_aruco_markers.py

Design notes Section 3 pick fiducials over YOLO digit classification for tray
identity: one frame gives both the ID and a full 6-DOF pose, with no training and
far less sensitivity to lighting and viewing angle.

Outputs
    models/aruco_marker_<n>/            model.sdf, model.config, texture
    ../../print/aruco_marker_<n>.png    600 dpi, print at exactly 100 mm

The printed black square must measure ``MARKER_MM`` edge to edge; that number is
``marker_length_m`` in sprayer_docking/config/docking.yaml, and getting it wrong
scales every range estimate.
"""
from __future__ import annotations

import os

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
REPO = os.path.dirname(os.path.dirname(PKG))
PRINT_DIR = os.path.join(REPO, 'print')

DICT = cv2.aruco.DICT_4X4_50
MARKER_MM = 100.0          # black square, edge to edge
QUIET_MM = 20.0            # white border; ArUco needs it to segment reliably
BOARD_MM = MARKER_MM + 2 * QUIET_MM
MARKER_Z = 0.30            # height of the marker centre above the floor
POST_GAP = 0.03            # clearance kept between the post's top and the board's bottom edge
DPI = 600

# tray centre -> marker face, matching worlds/nursery_room.world
TRAYS = [
    dict(n=1, name='baki_1_barat_laut', center=(-2.2, 2.2), dock_y=1.40),
    dict(n=2, name='baki_2_timur_laut', center=(2.2, 2.2), dock_y=1.40),
    dict(n=3, name='baki_3_barat_daya', center=(-2.6, -2.2), dock_y=-1.40),
    dict(n=4, name='baki_4_timur_daya', center=(2.6, -2.2), dock_y=-1.40),
]
TRAY_SX = 1.24             # rim lip, outer; see models/seedling_tray/model.sdf
TRAY_SY = 0.74
FACE_GAP = 0.02            # the stand sits this far clear of the rim

# THREE stands per tray, all carrying the tray's OWN id: the dock face plus both
# side faces. cv2.aruco only decodes inside roughly a +-20..25 deg cone off a
# marker's normal (myagv_sprayer_jetson/tools/measure_aruco_detection.py: 100% at
# +-20 deg, 0% at +-22 deg at 0.4 m), so a single face left each tray unreadable
# from the great majority of approach directions -- the "cannot see it from the
# side" problem.
#
# **The same id on all three faces, not one id per face.** A tray has a single
# identity: that is what `tray_waypoints.yaml`'s `marker_ids` means and what the
# YOLO detector's one-class-per-tray encodes, and physically you print the same
# marker three times.
#
# There is deliberately NO marker on the face opposite the dock side. Every tray
# sits in a room corner, so that face looks at a wall 0.41 m away, while standing
# inside its cone needs `safety_distance` 0.45 m plus the robot's 0.207 m
# circumradius = 0.657 m. A marker there would be geometrically unreadable.


def marker_image(marker_id: int, px: int = 512) -> np.ndarray:
    d = cv2.aruco.getPredefinedDictionary(DICT)
    try:
        img = cv2.aruco.generateImageMarker(d, marker_id, px)
    except AttributeError:                       # OpenCV < 4.7
        img = cv2.aruco.drawMarker(d, marker_id, px)
    quiet = int(px * QUIET_MM / MARKER_MM)
    return cv2.copyMakeBorder(img, quiet, quiet, quiet, quiet,
                              cv2.BORDER_CONSTANT, value=255)


def write_model(marker_id: int, out_dir: str):
    os.makedirs(os.path.join(out_dir, 'materials', 'textures'), exist_ok=True)
    os.makedirs(os.path.join(out_dir, 'materials', 'scripts'), exist_ok=True)
    name = f'aruco_marker_{marker_id}'
    tex = f'{name}.png'
    cv2.imwrite(os.path.join(out_dir, 'materials', 'textures', tex),
                marker_image(marker_id))

    with open(os.path.join(out_dir, 'materials', 'scripts', f'{name}.material'), 'w') as f:
        f.write(f'''material Aruco/Marker{marker_id}
{{
  technique
  {{
    pass
    {{
      lighting on
      ambient 0.85 0.85 0.85 1.0
      diffuse 1.0 1.0 1.0 1.0
      specular 0.02 0.02 0.02 1.0 1.0
      texture_unit
      {{
        texture {tex}
        filtering trilinear
      }}
    }}
  }}
}}
''')

    board = BOARD_MM / 1000.0
    with open(os.path.join(out_dir, 'model.config'), 'w') as f:
        f.write(f'''<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.7">model.sdf</sdf>
  <author><name>Deva</name><email>info@maorthonet.com</email></author>
  <description>ArUco DICT_4X4_50 id {marker_id}, {MARKER_MM:.0f} mm, on a small
  stand at the front face of tray {marker_id}. Model +x is the outward normal.</description>
</model>
''')

    # +x is the outward normal: the face the robot must approach from.
    #
    # The post must never overlap the board's z-span (MARKER_Z +/- board/2),
    # or its own radius pokes through the printed face from the front --
    # this was a real bug: with the post running the full height up to
    # MARKER_Z (the board's *centre*), its 8mm radius physically stuck out
    # past the board's front face (only 4mm ahead of the post's axis) for
    # the bottom half of the board, occluding enough bits that cv2.aruco
    # failed to decode the marker in ~99% of captured viewpoints. Stopping
    # the post POST_GAP below the board's bottom edge removes any z-overlap,
    # so no viewing angle can put the post in front of the printed face.
    post_len = MARKER_Z - board / 2.0 - POST_GAP
    with open(os.path.join(out_dir, 'model.sdf'), 'w') as f:
        f.write(f'''<?xml version="1.0"?>
<sdf version="1.7">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <!-- post: stops POST_GAP below the board's bottom edge, see comment above -->
      <visual name="post">
        <pose>0 0 {post_len / 2:.3f} 0 0 0</pose>
        <geometry><cylinder><radius>0.008</radius><length>{post_len:.3f}</length></cylinder></geometry>
        <material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.4 0.4 0.4 1</diffuse></material>
      </visual>
      <!-- printed sheet; the texture faces +x -->
      <visual name="board">
        <pose>0.004 0 {MARKER_Z:.3f} 0 0 0</pose>
        <geometry><box><size>0.002 {board:.3f} {board:.3f}</size></box></geometry>
        <material>
          <script>
            <uri>model://{name}/materials/scripts</uri>
            <uri>model://{name}/materials/textures</uri>
            <name>Aruco/Marker{marker_id}</name>
          </script>
        </material>
      </visual>
      <collision name="collision">
        <pose>0 0 {MARKER_Z / 2:.3f} 0 0 0</pose>
        <geometry><box><size>0.02 {board:.3f} {MARKER_Z:.3f}</size></box></geometry>
      </collision>
    </link>
  </model>
</sdf>
''')
    return name


def write_printable(marker_id: int):
    os.makedirs(PRINT_DIR, exist_ok=True)
    px = int(round(BOARD_MM / 25.4 * DPI))
    img = cv2.resize(marker_image(marker_id, px=int(px * MARKER_MM / BOARD_MM)),
                     (px, px), interpolation=cv2.INTER_NEAREST)
    canvas = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    cv2.putText(canvas, f'ArUco DICT_4X4_50  id={marker_id}  '
                        f'black square = {MARKER_MM:.0f} mm',
                (20, px - 25), cv2.FONT_HERSHEY_SIMPLEX, 1.4, (120, 120, 120), 3)
    path = os.path.join(PRINT_DIR, f'aruco_marker_{marker_id}.png')
    cv2.imwrite(path, canvas)
    return path


def world_poses(tray):
    """The three stand poses for one tray: dock face, then both side faces.

    Returns ``[(face_name, x, y, yaw), ...]``. The model's +x is its printed
    normal, so ``yaw`` is simply the direction that face looks in. All three
    carry tray ``n``'s own marker id -- see the note above.
    """
    cx, cy = tray['center']
    ny = 1.0 if tray['dock_y'] > cy else -1.0   # +1 = dock side lies north
    dy = TRAY_SY / 2.0 + FACE_GAP
    dx = TRAY_SX / 2.0 + FACE_GAP
    half_pi = 1.5708
    return [
        ('dock',  cx,      cy + ny * dy, half_pi if ny > 0 else -half_pi),
        ('left',  cx - dx, cy,           3.1416),
        ('right', cx + dx, cy,           0.0),
    ]


def main():
    print(f'marker {MARKER_MM:.0f} mm + {QUIET_MM:.0f} mm quiet zone '
          f'= board {BOARD_MM:.0f} mm')
    lines = []
    for t in TRAYS:
        out_dir = os.path.join(PKG, 'models', f'aruco_marker_{t["n"]}')
        write_model(t['n'], out_dir)
        p = write_printable(t['n'])
        print(f'  tray {t["n"]} -> model + {os.path.relpath(p, REPO)} '
              f'(print 3x, one per face)')
        for face, x, y, yaw in world_poses(t):
            lines.append(f'    <include>\n'
                         f'      <uri>model://aruco_marker_{t["n"]}</uri>\n'
                         f'      <name>marker_{t["name"]}_{face}</name>\n'
                         f'      <pose>{x:.3f} {y:.3f} 0 0 0 {yaw}</pose>\n'
                         f'    </include>')
            print(f'      {face:5s} ({x:+.3f}, {y:+.3f}) yaw {yaw:+.4f}')
    print('\nSDF snippet for worlds/nursery_room.world:\n')
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
