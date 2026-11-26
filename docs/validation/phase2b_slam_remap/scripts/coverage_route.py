#!/usr/bin/env python3
"""
Coverage route for the Phase 2b mapping drive (odometry frame = spawn-relative world frame).

Waypoints were designed with the Gazebo world geometry (route design only; no occupancy data is
taken from it). `check` verifies the robot-centre clearance of every segment against the true
LiDAR-height geometry raster, and renders the route.

Usage: coverage_route.py check <workspace> <out_dir>
"""
import math
import sys

# (x, y, action) in the odom frame (robot spawns at the odom origin, yaw 0). action: None or 'spin'
ROUTE = [
    (0.00, 0.00, 'spin'),                                     # spawn area
    (-1.50, 0.40, None), (-3.60, 0.60, None),                # crossing y ~ 0.5 to the west aisle
    (-3.70, 3.00, None), (-3.70, 6.50, None),                # west aisle north
    (-3.65, 8.80, 'spin'),                                    # home dock approach area
    (-3.40, 6.90, None), (-1.50, 6.85, None),                # north crossing (between block and legs)
    (0.50, 7.10, None), (1.60, 7.60, None),
    (1.80, 8.90, 'spin'),                                     # loading zone approach area
    (1.70, 7.00, None), (1.70, 4.50, None), (1.70, 1.00, None),   # east aisle south
    (1.70, -3.00, None), (1.75, -6.00, None),
    (1.75, -8.70, 'spin'),                                    # inventory approach area
    (0.20, -8.75, None), (-0.95, -8.85, None),               # south crossing: equidistant line between
    (-1.60, -9.15, None),                                     #   block corner and pallet jack (~0.38 m max)
    (-3.60, -9.10, 'spin'),                                   # dispatch approach area
    (-3.80, -6.00, None), (-3.80, -2.00, None),              # west aisle north (loop 1 closes)
    (-3.60, 0.60, None), (-1.50, 0.40, None),
    (0.40, 0.60, None), (0.40, 1.50, None),                  # around the central block corner
    (0.40, 4.00, None), (0.30, 6.30, None),                  # middle corridor north
    (-0.50, 6.85, None),                                      # joins the north crossing (loop 2)
    (0.40, 6.30, None), (0.50, 4.00, None), (0.80, 1.00, None),   # back down the middle corridor
    (0.80, -3.00, None), (0.20, -6.50, None),                # southern central traverse
    (0.80, -8.40, None), (1.70, -6.00, None),                # joins east aisle (loop 3)
    (1.70, -1.00, None), (0.00, 0.00, 'spin'),               # back to spawn
]

# Targeted additional coverage (attempt 2): the west aisle stayed low-confidence after one pass at
# 0.15 m/s (Cartographer cells 35-49, saved as unknown). Two more slow passes with spins.
WEST_AISLE_EXTENSION = [
    (-1.50, 0.40, None), (-3.60, 0.60, 'spin'),
    (-3.70, -2.00, None), (-3.75, -4.00, 'spin'), (-3.80, -6.00, None), (-3.60, -8.60, None),
    (-3.80, -6.00, None), (-3.75, -4.00, None), (-3.70, -2.00, None), (-3.60, 0.60, None),
    (-3.70, 3.00, None), (-3.70, 4.00, 'spin'), (-3.70, 6.50, None), (-3.60, 8.60, None),
    (-3.70, 6.50, None), (-3.70, 3.00, None), (-3.60, 0.60, None),
    (-1.50, 0.40, None), (0.00, 0.00, 'spin'),
]
ROUTE = ROUTE + WEST_AISLE_EXTENSION


def check(ws, out):
    import numpy as np
    import cv2
    gt = np.load(ws + '/diagnostics_20261125/static/gt_body.npy') > 0
    H, W = gt.shape
    res, ox, oy = 0.05, -7.56, -10.8
    d = cv2.distanceTransform(255 - gt.astype(np.uint8) * 255, cv2.DIST_L2, 5) * res
    lines, worst, total = [], (9, None), 0.0
    for (x0, y0, _), (x1, y1, a1) in zip(ROUTE, ROUTE[1:]):
        L = math.hypot(x1 - x0, y1 - y0)
        total += L
        n = max(2, int(L / 0.02))
        mins = []
        for k in range(n + 1):
            x, y = x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n
            c = int(math.floor((x - ox) / res))
            r = H - 1 - int(math.floor((y - oy) / res))
            mins.append(d[r, c])
        m = min(mins)
        lines.append(f'  ({x0:+.2f},{y0:+.2f}) -> ({x1:+.2f},{y1:+.2f}) {L:5.2f} m  min clearance {m:.2f} m'
                     + ('  [spin at end]' if a1 == 'spin' else ''))
        if m < worst[0]:
            worst = (m, (x1, y1))
    spins = [p for p in ROUTE if p[2] == 'spin']
    lines.append(f'route length {total:.1f} m, {len(spins)} spins; worst segment clearance {worst[0]:.2f} m near {worst[1]}')
    for x, y, _ in spins:
        c = int(math.floor((x - ox) / res)); r = H - 1 - int(math.floor((y - oy) / res))
        lines.append(f'  spin at ({x:+.2f},{y:+.2f}): clearance {d[r, c]:.2f} m')
    print('\n'.join(lines))
    open(out + '/route_check.txt', 'w').write('\n'.join(lines) + '\n')
    vis = cv2.cvtColor((255 - gt * 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
    pix = [(int((x - ox) / res), H - 1 - int((y - oy) / res)) for x, y, _ in ROUTE]
    for p, q in zip(pix, pix[1:]):
        cv2.line(vis, p, q, (200, 60, 0), 1)
    for (x, y, a), p in zip(ROUTE, pix):
        if a == 'spin':
            cv2.circle(vis, p, 4, (0, 0, 255), 1)
    cv2.imwrite(out + '/coverage_route.png', cv2.resize(vis, (W * 3, H * 3), interpolation=cv2.INTER_NEAREST))


if __name__ == '__main__' and sys.argv[1:2] == ['check']:
    check(sys.argv[2], sys.argv[3])
