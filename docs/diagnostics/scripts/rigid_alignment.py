#!/usr/bin/env python3
"""Rigid (SE2) map<->world alignment, and what remains after removing it.

Usage: rigid_alignment.py <workspace> <static_dir> <run_dir>
Fits (a) map obstacles to true LiDAR-height geometry (chamfer, grid search + refinement),
(b) AMCL poses (map frame) to Gazebo truth (world frame) by least squares (Umeyama 2D).
Reports residuals by region and decomposes station positioning errors.
"""
import math, sys, numpy as np, cv2
ws, static, run = sys.argv[1:4]
res, ox, oy = 0.05, -7.56, -10.8
img = cv2.imread(ws + '/src/warehouse_bringup/maps/warehouse_map.pgm', cv2.IMREAD_UNCHANGED)
H, W = img.shape
occ = (1 - img / 255.0) > 0.65
gt = np.load(static + '/gt_body.npy') > 0
dgt = cv2.distanceTransform(255 - gt.astype(np.uint8) * 255, cv2.DIST_L2, 5) * res
my, mx = np.nonzero(occ); MX = ox + mx * res; MY = oy + (H - 1 - my) * res

def to_world(th, tx, ty, X, Y):           # world = R(th) * map + t
    c, s = math.cos(th), math.sin(th); return c * X - s * Y + tx, s * X + c * Y + ty
def score(p, X=MX, Y=MY):
    wx, wy = to_world(*p, X, Y)
    c = np.round((wx - ox) / res).astype(int); r = np.round(H - 1 - (wy - oy) / res).astype(int)
    ok = (c >= 0) & (c < W) & (r >= 0) & (r < H); d = np.full(len(c), 0.5); d[ok] = np.minimum(dgt[r[ok], c[ok]], 0.5)
    return d
best = None
for th in np.radians(np.arange(-4, 4.01, 0.25)):
    for tx in np.arange(-0.6, 0.61, 0.05):
        for ty in np.arange(-0.6, 0.61, 0.05):
            m = score((th, tx, ty)).mean()
            if best is None or m < best[0]: best = (m, (th, tx, ty))
p = np.array(best[1]); step = np.array([math.radians(0.1), 0.01, 0.01])
for _ in range(4):
    for i in range(3):
        for dlt in (-1, 1):
            q = p.copy(); q[i] += dlt * step[i]
            while score(q).mean() < score(p).mean(): p = q.copy(); q[i] += dlt * step[i]
    step /= 2
d0, d1 = score((0, 0, 0)), score(tuple(p))
out = []
P = lambda s: (out.append(s), print(s))
P('A. Geometry-based rigid fit: world = R*map + t')
P(f'   theta {math.degrees(p[0]):+.2f} deg, t = ({p[1]:+.3f}, {p[2]:+.3f}) m')
P(f'   map-obstacle to true-geometry distance: identity mean {d0.mean():.3f} / within 10 cm {100*np.mean(d0<=0.1):.0f}%  ->  rigid-fit mean {d1.mean():.3f} / within 10 cm {100*np.mean(d1<=0.1):.0f}%')
for lo, hi in [(-11, -6), (-6, -2), (-2, 2), (2, 6), (6, 11)]:
    sel = (MY >= lo) & (MY < hi)
    # local best translation after global rigid fit
    wx, wy = to_world(*p, MX[sel], MY[sel])
    bestl = min(((score((0, a, b), wx, wy).mean(), a, b) for a in np.arange(-0.5, 0.51, 0.05) for b in np.arange(-0.5, 0.51, 0.05)))
    P(f'   map y in [{lo:+3d},{hi:+3d}): within 10 cm identity {100*np.mean(d0[sel]<=0.1):3.0f}% rigid {100*np.mean(d1[sel]<=0.1):3.0f}%; remaining local shift after rigid fit ({bestl[1]:+.2f},{bestl[2]:+.2f}) m')

e = np.loadtxt(run + '/analysis/amcl_vs_truth.csv', delimiter=',')
tx_, ty_ = e[:, 5], e[:, 6]; ax_, ay_ = tx_ + e[:, 2], ty_ + e[:, 3]
A = np.c_[ax_, ay_]; B = np.c_[tx_, ty_]                     # fit truth = R*amcl + t
ca, cb = A.mean(0), B.mean(0); U, S, Vt = np.linalg.svd((A - ca).T @ (B - cb))
R = (U @ Vt).T
if np.linalg.det(R) < 0: Vt[-1] *= -1; R = (U @ Vt).T
t = cb - R @ ca; th = math.atan2(R[1, 0], R[0, 0])
resid = B - (A @ R.T + t); rn = np.hypot(*resid.T); raw = np.hypot(e[:, 2], e[:, 3])
P('\nB. Pose-based rigid fit (truth = R*AMCL + t, all 252 AMCL updates)')
P(f'   theta {math.degrees(th):+.2f} deg, t = ({t[0]:+.3f}, {t[1]:+.3f}) m')
P(f'   position error: raw median {np.median(raw):.3f} p95 {np.percentile(raw,95):.3f} max {raw.max():.3f} m -> after rigid fit median {np.median(rn):.3f} p95 {np.percentile(rn,95):.3f} max {rn.max():.3f} m')
for lo, hi in [(-11, -6), (-6, -2), (-2, 2), (2, 6), (6, 11)]:
    s = (ty_ >= lo) & (ty_ < hi)
    if s.sum(): P(f'   true y in [{lo:+3d},{hi:+3d}): raw median {np.median(raw[s]):.3f} m, after rigid fit median {np.median(rn[s]):.3f} m (mean residual {resid[s,0].mean():+.2f},{resid[s,1].mean():+.2f})')
np.savetxt(run + '/analysis/amcl_rigid_residual.csv', np.c_[e[:, 0], tx_, ty_, raw, rn], delimiter=',', fmt='%.4f', header='sim_stamp,true_x,true_y,raw_err,err_after_rigid_fit')

P('\nC. Station positioning: true stop vs intended goal, and the part a rigid frame offset explains')
stops = {'Home Charging Dock': ((-4.132, 9.080), (-3.678, 9.3)), 'Loading Zone': ((1.399, 9.308), (1.803, 9.3)),
         'Inventory Scan Station': ((1.969, -8.948), (1.801, -9.0)), 'Dispatch Zone': ((-3.338, -9.420), (-3.630, -9.3))}
for n, (true, goal) in stops.items():
    g = np.array(goal); tr = np.array(true)
    pred_rigid_A = np.array(to_world(*p, g[0], g[1]))       # where a goal given in map frame lands in world, rigid geometry fit
    pred_rigid_B = R @ g + t
    P(f'   {n:22s} miss {np.linalg.norm(tr-g):.3f} m | rigid model (geometry) predicts miss {np.linalg.norm(pred_rigid_A-g):.3f} m, residual {np.linalg.norm(tr-pred_rigid_A):.3f} m '
      f'| rigid model (poses) predicts {np.linalg.norm(pred_rigid_B-g):.3f} m, residual {np.linalg.norm(tr-pred_rigid_B):.3f} m')
open(run + '/analysis/rigid_alignment.txt', 'w').write('\n'.join(out) + '\n')
