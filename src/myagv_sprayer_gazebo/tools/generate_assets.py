#!/usr/bin/env python3
"""Generate the textures used by the nursery_room world and the Nav2 map.

Run once (the outputs are committed, so this is only needed if you change
the room dimensions in ROOM_* below):

    python3 tools/generate_assets.py

Outputs
  models/nursery_floor/materials/textures/floor_tiles.png
  models/seedling_tray/materials/textures/seedlings.png
  ../myagv_sprayer_navigation/maps/nursery_room.pgm / .yaml
"""
import os
import random

from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
NAV_MAPS = os.path.join(PKG, '..', 'myagv_sprayer_navigation', 'maps')

# ---- room geometry (must match worlds/nursery_room.world) -------------------
ROOM_X = 8.0            # east-west (m)
ROOM_Y = 6.0            # north-south (m)
WALL_T = 0.15
TRAY_SX, TRAY_SY = 1.20, 0.70
TRAYS = [               # (x, y) of tray centres, map frame = room centre
    (-2.2,  2.2), ( 2.2,  2.2),
    (-2.6, -2.2), ( 2.6, -2.2),
]
DOOR_W = 1.2            # south wall opening centred on x = 0


def floor_tiles(path, px=512, tiles=2):
    """Light-grey porcelain tiles with darker grout, seamless."""
    img = Image.new('RGB', (px, px), (206, 206, 200))
    d = ImageDraw.Draw(img)
    step = px // tiles
    rnd = random.Random(7)
    for i in range(tiles):
        for j in range(tiles):
            shade = rnd.randint(-8, 8)
            c = (214 + shade, 213 + shade, 206 + shade)
            d.rectangle([i * step + 3, j * step + 3, (i + 1) * step - 3, (j + 1) * step - 3], fill=c)
    # grout lines
    for k in range(tiles + 1):
        d.line([(k * step, 0), (k * step, px)], fill=(165, 165, 160), width=4)
        d.line([(0, k * step), (px, k * step)], fill=(165, 165, 160), width=4)
    img.save(path)


def seedlings(path, px=512):
    """Top view of a mat-type rice nursery tray: dark moist soil + green blades."""
    img = Image.new('RGB', (px, px), (74, 52, 32))
    d = ImageDraw.Draw(img)
    rnd = random.Random(3)
    # soil noise
    for _ in range(6000):
        x, y = rnd.randrange(px), rnd.randrange(px)
        v = rnd.randint(-14, 14)
        d.point((x, y), fill=(74 + v, 52 + v, 32 + v))
    # rows of blades (rows run along the tray's long axis = image x)
    for row in range(14):
        y0 = int((row + 0.5) * px / 14)
        for _ in range(60):
            x = rnd.randrange(px)
            h = rnd.randint(18, 40)
            g = rnd.randint(120, 200)
            col = (rnd.randint(30, 70), g, rnd.randint(30, 60))
            d.line([(x, y0 + 8), (x + rnd.randint(-4, 4), y0 - h)], fill=col, width=rnd.randint(1, 3))
    img.save(path)


def nav_map(res=0.05, margin=1.0):
    w = int((ROOM_X + 2 * margin) / res)
    h = int((ROOM_Y + 2 * margin) / res)
    img = Image.new('L', (w, h), 205)          # unknown
    d = ImageDraw.Draw(img)

    def to_px(x, y):
        # map origin (bottom-left) at (-ROOM_X/2 - margin, -ROOM_Y/2 - margin)
        px = (x + ROOM_X / 2 + margin) / res
        py = h - (y + ROOM_Y / 2 + margin) / res
        return px, py

    # free interior
    d.rectangle([to_px(-ROOM_X / 2, ROOM_Y / 2), to_px(ROOM_X / 2, -ROOM_Y / 2)], fill=254)

    def wall(x0, y0, x1, y1):
        d.rectangle([to_px(min(x0, x1), max(y0, y1)), to_px(max(x0, x1), min(y0, y1))], fill=0)

    hx, hy, t = ROOM_X / 2, ROOM_Y / 2, WALL_T
    wall(-hx - t, hy, hx + t, hy + t)                 # north
    wall(-hx - t, -hy - t, hx + t, -hy)               # south (full)
    d.rectangle([to_px(-DOOR_W / 2, -hy), to_px(DOOR_W / 2, -hy - t)], fill=254)  # door opening
    wall(-hx - t, -hy, -hx, hy)                       # west
    wall(hx, -hy, hx + t, hy)                         # east
    for (x, y) in TRAYS:
        wall(x - TRAY_SX / 2, y - TRAY_SY / 2, x + TRAY_SX / 2, y + TRAY_SY / 2)
    # wall fans / climate sensor hang at 1.35-1.7 m, above the LiDAR plane,
    # so they are deliberately NOT drawn into the static map.

    os.makedirs(NAV_MAPS, exist_ok=True)
    pgm = os.path.join(NAV_MAPS, 'nursery_room.pgm')
    img.save(pgm)
    with open(os.path.join(NAV_MAPS, 'nursery_room.yaml'), 'w') as f:
        f.write(
            'image: nursery_room.pgm\n'
            f'resolution: {res}\n'
            f'origin: [{-ROOM_X / 2 - margin:.3f}, {-ROOM_Y / 2 - margin:.3f}, 0.0]\n'
            'negate: 0\n'
            'occupied_thresh: 0.65\n'
            'free_thresh: 0.196\n'
        )
    print('map', w, 'x', h, 'px ->', pgm)


if __name__ == '__main__':
    floor_tiles(os.path.join(PKG, 'models', 'nursery_floor', 'materials', 'textures', 'floor_tiles.png'))
    seedlings(os.path.join(PKG, 'models', 'seedling_tray', 'materials', 'textures', 'seedlings.png'))
    nav_map()
    print('done')
