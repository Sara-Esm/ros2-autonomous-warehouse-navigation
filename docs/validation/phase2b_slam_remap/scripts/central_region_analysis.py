#!/usr/bin/env python3
"""Does the central unknown region of warehouse_map_v2 matter for the mission routes? (evaluation only)

Shortest collision-free routes on the TRUE geometry (0.19 m centre clearance, grid Dijkstra) between the
intended approach poses, mapped into the v2 frame with the evaluation fit, checked against v2 unknown cells
and the 'central region' box; compared with the real-NavFn strict-policy plan lengths (pre-mission check).
"""
import heapq, math, re, sys
import cv2, numpy as np
ws = sys.argv[1]
sys.path.insert(0, ws + '/src/warehouse_bringup')
from warehouse_bringup.stations import STATIONS_WORLD  # noqa: E402
gt = np.load(ws + '/diagnostics_20261125/static/gt_body.npy') > 0; GH, GW = gt.shape
yy, xx = np.mgrid[0:GH, 0:GW]; X = -7.56 + (xx + 0.5) * 0.05; Y = -10.8 + (GH - 1 - yy + 0.5) * 0.05
blk = gt | (X < -6.9) | (X > 6.9) | (Y < -10.35) | (Y > 10.35)
free = cv2.distanceTransform((~blk).astype(np.uint8) * 255, cv2.DIST_L2, 5) * 0.05 > 0.19
cell = lambda x, y: (GH - 1 - int(math.floor((y + 10.8) / 0.05)), int(math.floor((x + 7.56) / 0.05)))
def path(a, b):
    s, t = cell(*a), cell(*b); D = {s: 0.0}; prev = {}; pq = [(0.0, s)]
    while pq:
        d0, u = heapq.heappop(pq)
        if u == t: break
        if d0 > D[u]: continue
        for dr, dc, w in ((1,0,1),(-1,0,1),(0,1,1),(0,-1,1),(1,1,1.4142),(1,-1,1.4142),(-1,1,1.4142),(-1,-1,1.4142)):
            v = (u[0] + dr, u[1] + dc)
            if 0 <= v[0] < GH and 0 <= v[1] < GW and free[v] and d0 + w < D.get(v, 1e18):
                D[v] = d0 + w; prev[v] = u; heapq.heappush(pq, (d0 + w, v))
    p = [t]
    while p[-1] in prev: p.append(prev[p[-1]])
    return D[t] * 0.05, [(-7.56 + (c + 0.5) * 0.05, -10.8 + (GH - 1 - r + 0.5) * 0.05) for r, c in p[::-1]]
img = cv2.imread(ws + '/src/warehouse_bringup/maps/warehouse_map_v2.pgm', cv2.IMREAD_UNCHANGED); H, W = img.shape
th, tx, ty = math.radians(0.71), -0.009, 0.040
def v2cell(x, y):
    dx, dy = x - tx, y - ty; mx = math.cos(th) * dx + math.sin(th) * dy; my = -math.sin(th) * dx + math.cos(th) * dy
    return H - 1 - math.floor((my + 10.7) / 0.05), math.floor((mx + 7.28) / 0.05)
central = lambda x, y: -3.0 <= x <= 1.0 and -5.5 <= y <= 0.0      # the large unknown blob (world frame)
cen = np.array([[central(-7.56 + (c + .5) * .05, -10.8 + (GH - 1 - r + .5) * .05) for c in range(GW)] for r in range(GH)])
print(f'central box x[-3,1] y[-5.5,0]: {100*np.mean([img[v2cell(x,y)]==205 for x in np.arange(-3,1,0.1) for y in np.arange(-5.5,0,0.1)]):.0f}% unknown in v2')
plans = {}
for l in open(ws + '/diagnostics_phase2b/planner_check_surveyed.txt'):
    m = re.search(r'(\w+)\s+->\s+(\w+)\s*: plan\s+([\d.]+) m', l)
    if m: plans[(m.group(1), m.group(2))] = float(m.group(3))
pts = {'spawn': (0.0, 0.0)}; pts.update({k: v[:2] for k, v in STATIONS_WORLD.items()})
print('| Leg | True-geometry shortest | through central box | on v2 unknown cells | Strict NavFn plan (v2) | Plan / shortest |')
print('|---|---|---|---|---|---|')
for a, b in [('spawn', 'home_dock'), ('home_dock', 'loading_zone'), ('loading_zone', 'inventory_scan'), ('inventory_scan', 'dispatch_zone'), ('dispatch_zone', 'home_dock')]:
    L, pp = path(pts[a], pts[b])
    seg = [math.dist(p, q) for p, q in zip(pp, pp[1:])]
    inc = sum(s for s, p in zip(seg, pp[1:]) if central(*p)); unk = sum(s for s, p in zip(seg, pp[1:]) if img[v2cell(*p)] == 205)
    pl = plans.get((a, b), float('nan'))
    print(f'| {a} -> {b} | {L:.2f} m | {inc:.2f} m | {unk:.2f} m | {pl:.2f} m | {pl/L:.2f} |')
