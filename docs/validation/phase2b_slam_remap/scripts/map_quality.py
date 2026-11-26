#!/usr/bin/env python3
"""
Evaluate a SLAM map against the true world geometry (evaluation only; nothing is written to the map).

Usage: map_quality.py <workspace> <map_pgm> <map_yaml> <out_dir> <label>

Reports: pixel classes; rigid world=R*map+t fit (map obstacles -> true LiDAR-height geometry) with
regional residual shifts; coverage of mission corridors (world-frame polylines mapped into the map
frame with the fitted transform); strict-policy (allow_unknown=false) reachability between spawn and
the intended station approach poses under Nav2 conventions; exterior connectivity.
"""
import heapq, math, sys
import cv2, numpy as np, yaml
ws, pgm, myaml, out, label = sys.argv[1:6]
sys.path.insert(0, ws + '/src/warehouse_bringup')
from warehouse_bringup.stations import STATIONS_WORLD  # noqa: E402  (intended poses, evaluation reference)
img = cv2.imread(pgm, cv2.IMREAD_UNCHANGED)
meta = yaml.safe_load(open(myaml))
H, W = img.shape; RES = float(meta['resolution']); OX, OY = float(meta['origin'][0]), float(meta['origin'][1])
L = []
P = lambda s='': (L.append(s), print(s))
vals, cnts = np.unique(img, return_counts=True)
P(f'# {label}: {W}x{H} px @ {RES} m, origin ({OX:.3f},{OY:.3f}), yaw {meta["origin"][2]}')
P(f'pixel values: {dict(zip(vals.tolist(), cnts.tolist()))}')
occ = img < 50; free = img > 250; unk = ~(occ | free)
P(f'classes (PGM): occupied {occ.sum()}, free {free.sum()}, unknown {unk.sum()} ({100*unk.mean():.1f}% unknown)')

gt = np.load(ws + '/diagnostics_20261125/static/gt_body.npy') > 0          # world frame raster
GH, GW = gt.shape; GOX, GOY = -7.56, -10.8
dgt = cv2.distanceTransform(255 - gt.astype(np.uint8) * 255, cv2.DIST_L2, 5) * 0.05
r_, c_ = np.nonzero(occ)
MX = OX + (c_ + 0.5) * RES; MY = OY + (H - 1 - r_ + 0.5) * RES


def to_world(p, X, Y):
    th, tx, ty = p
    return math.cos(th) * X - math.sin(th) * Y + tx, math.sin(th) * X + math.cos(th) * Y + ty


def score(p, X=MX, Y=MY):
    wx, wy = to_world(p, X, Y)
    c = np.floor((wx - GOX) / 0.05).astype(int); r = GH - 1 - np.floor((wy - GOY) / 0.05).astype(int)
    ok = (c >= 0) & (c < GW) & (r >= 0) & (r < GH)
    d = np.full(len(c), 0.5); d[ok] = np.minimum(dgt[r[ok], c[ok]], 0.5)
    return d


best = None
for th in np.radians(np.arange(-4, 4.01, 0.25)):
    for tx in np.arange(-0.6, 0.61, 0.05):
        for ty in np.arange(-0.6, 0.61, 0.05):
            m = score((th, tx, ty)).mean()
            if best is None or m < best[0]:
                best = (m, np.array([th, tx, ty]))
p = best[1]; step = np.array([math.radians(0.1), 0.01, 0.01])
for _ in range(4):
    for i in range(3):
        for s in (-1, 1):
            q = p.copy(); q[i] += s * step[i]
            while score(q).mean() < score(p).mean():
                p = q.copy(); q[i] += s * step[i]
    step /= 2
d0, d1 = score((0, 0, 0)), score(tuple(p))
P(f'rigid fit world = R*map + t: theta {math.degrees(p[0]):+.2f} deg, t ({p[1]:+.3f},{p[2]:+.3f}) m')
P(f'map obstacles within 10 cm of true geometry: identity {100*np.mean(d0<=0.1):.0f}%, rigid fit {100*np.mean(d1<=0.1):.0f}%; '
  f'mean distance {d0.mean():.3f} -> {d1.mean():.3f} m')
for lo, hi in [(-11, -6), (-6, -2), (-2, 2), (2, 6), (6, 11)]:
    sel = (MY >= lo) & (MY < hi)
    if sel.sum() < 30:
        continue
    wx, wy = to_world(tuple(p), MX[sel], MY[sel])
    def sc(a, b):
        c = np.floor((wx + a - GOX) / 0.05).astype(int); r = GH - 1 - np.floor((wy + b - GOY) / 0.05).astype(int)
        ok = (c >= 0) & (c < GW) & (r >= 0) & (r < GH)
        d = np.full(len(c), 0.5); d[ok] = np.minimum(dgt[r[ok], c[ok]], 0.5); return d.mean()
    bl = min((sc(a, b), a, b) for a in np.arange(-0.3, 0.31, 0.05) for b in np.arange(-0.3, 0.31, 0.05))
    P(f'  map y [{lo:+3d},{hi:+3d}): within 10 cm after rigid fit {100*np.mean(d1[sel]<=0.1):3.0f}%; residual local shift ({bl[1]:+.2f},{bl[2]:+.2f}) m')


def world_to_map(x, y):
    th, tx, ty = p
    dx, dy = x - tx, y - ty
    return math.cos(th) * dx + math.sin(th) * dy, -math.sin(th) * dx + math.cos(th) * dy


def cell(x, y):
    return H - 1 - math.floor((y - OY) / RES), math.floor((x - OX) / RES)


CORRIDORS = {   # world-frame polylines of the mission corridors (route-design reference)
    'spawn area': [(-0.5, 0.0), (0.5, 0.0)],
    'west aisle': [(-3.7, -8.5), (-3.7, 8.5)],
    'east aisle': [(1.7, -8.4), (1.7, 8.6)],
    'north crossing': [(-3.4, 6.9), (-1.5, 6.85), (0.5, 7.1), (1.6, 7.6)],
    'south crossing': [(1.6, -8.75), (0.2, -8.75), (-0.95, -8.85), (-1.6, -9.15), (-3.5, -9.1)],
    'central crossing y~0.5': [(-3.5, 0.6), (-1.5, 0.4), (0.4, 0.6), (1.6, 0.8)],
    'middle corridor': [(0.4, 0.6), (0.4, 6.3)],
    'dock approach': [(-3.678, 8.6), (-3.678, 9.3)],
    'loading approach': [(1.803, 8.6), (1.803, 9.3)],
    'inventory approach': [(1.801, -8.3), (1.801, -9.0)],
    'dispatch approach': [(-3.63, -8.6), (-3.63, -9.3)],
}
dist = cv2.distanceTransform(255 - occ.astype(np.uint8) * 255, cv2.DIST_L2, 5) * RES
P('corridor coverage (fraction of sampled corridor cells that are known free / known free with >0.14 m clearance):')
for name, pl in CORRIDORS.items():
    tot = kf = kt = 0
    for (x0, y0), (x1, y1) in zip(pl, pl[1:]):
        n = max(2, int(math.hypot(x1 - x0, y1 - y0) / 0.05))
        for k in range(n + 1):
            mx, my = world_to_map(x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n)
            r, c = cell(mx, my)
            if 0 <= r < H and 0 <= c < W:
                tot += 1; kf += free[r, c]; kt += free[r, c] and dist[r, c] > 0.14
    P(f'  {name:24s} {100*kf/tot:5.1f}% / {100*kt/tot:5.1f}%')

trav = free & (dist > 0.14)
n, lab = cv2.connectedComponents(trav.astype(np.uint8), connectivity=8)
pts = {'spawn': world_to_map(0.0, 0.0)}
pts.update({k: world_to_map(*v[:2]) for k, v in STATIONS_WORLD.items()})
P('strict-policy reachability (known free, clearance > 0.14 m), intended poses mapped with the evaluation fit:')
labels = {}
for k, (x, y) in pts.items():
    r, c = cell(x, y)
    lb = lab[r, c]
    if lb == 0:
        ys, xs = np.nonzero(trav); j = np.argmin(np.hypot(ys - r, xs - c))
        P(f'  {k:15s} cell blocked/unknown; nearest traversable {math.hypot(ys[j]-r, xs[j]-c)*RES:.2f} m away')
        lb = lab[ys[j], xs[j]]
    labels[k] = lb
for lb in sorted(set(labels.values())):
    comp = lab == lb
    ext = bool(comp[0].any() or comp[-1].any() or comp[:, 0].any() or comp[:, -1].any())
    P(f'  region {lb}: {comp.sum()*RES*RES:.1f} m^2, contains {[k for k, v in labels.items() if v == lb]}, touches map border: {ext}')
P(f'  all mission points in one region: {len(set(labels.values())) == 1}')
open(out + '/map_quality.txt', 'w').write('\n'.join(L) + '\n')
vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
cv2.imwrite(out + '/map_view.png', cv2.resize(vis, (W * 2, H * 2), interpolation=cv2.INTER_NEAREST))
