#!/usr/bin/env python3
"""Regional rigid offsets between map obstacles and GT geometry (map distortion)."""
import sys, math, numpy as np, cv2
ws, out = sys.argv[1], sys.argv[2]
img = cv2.imread(ws + '/src/warehouse_bringup/maps/warehouse_map.pgm', cv2.IMREAD_UNCHANGED)
vals, cnts = np.unique(img, return_counts=True)
print('pgm value histogram:', dict(zip(vals.tolist(), cnts.tolist())))
for v in vals:
    occ = 1 - v / 255
    print(f'  value {v}: occ={occ:.3f} -> {"OCCUPIED" if occ > 0.65 else "FREE" if occ < 0.25 else "UNKNOWN"} (map_server trinary, thresholds 0.65/0.25)')
H, W = img.shape; res = 0.05; ox, oy = -7.56, -10.8
occ = (1 - img / 255) > 0.65
gt = np.load(out + '/gt_body.npy') > 0
dgt = cv2.distanceTransform(255 - gt.astype(np.uint8) * 255, cv2.DIST_L2, 5) * res
my, mx = np.nonzero(occ)
X = ox + mx * res; Y = oy + (H - 1 - my) * res
def search(sel):
    best = None
    for dx in np.arange(-0.6, 0.61, 0.05):
        for dy in np.arange(-0.6, 0.61, 0.05):
            c = np.round((X[sel] - dx - ox) / res).astype(int); r = np.round(H - 1 - (Y[sel] - dy - oy) / res).astype(int)
            ok = (c >= 0) & (c < W) & (r >= 0) & (r < H)
            d = np.full(len(c), 1.0); d[ok] = np.minimum(dgt[r[ok], c[ok]], 1.0)
            s = (np.mean(d <= 0.075), -np.median(d))
            if best is None or s > best[0]: best = (s, dx, dy, np.median(d))
    return best
print('\nRegional translation of map obstacles relative to true geometry (map - world):')
for lo, hi in [(-10.8, -6), (-6, -2), (-2, 2), (2, 6), (6, 11.2)]:
    for xlo, xhi, nm in [(-8, -1, 'west'), (-1, 9, 'east')]:
        sel = (Y >= lo) & (Y < hi) & (X >= xlo) & (X < xhi)
        if sel.sum() < 40: continue
        (frac, _), dx, dy, med = search(sel)
        c0 = np.round((X[sel] - ox) / res).astype(int); r0 = np.round(H - 1 - (Y[sel] - oy) / res).astype(int)
        med0 = np.median(dgt[r0, c0])
        print(f'  y[{lo:+5.1f},{hi:+5.1f}) {nm:4s}: n={sel.sum():4d} raw median dist={med0:.3f} m | best shift dx={dx:+.2f} dy={dy:+.2f} -> {100*frac:.0f}% within 7.5 cm, median {med:.3f} m')
# west wall line fit: map occupied cells with x < -6 and y in (-10, 10)
sel = (X < -5.9) & (X > -7.3) & (Y > -9.5) & (Y < 9.5)
A = np.polyfit(Y[sel], X[sel], 1)
print(f'\nWest wall in map: x = {A[0]:+.4f}*y {A[1]:+.3f}  -> angle {math.degrees(math.atan(A[0])):+.2f} deg; x at y=-9: {np.polyval(A,-9):+.2f}, at y=+9: {np.polyval(A,9):+.2f}')
gsel = gt & (np.arange(W)[None, :] < (-5.9 - ox) / res)
gy, gx = np.nonzero(gsel); GX = ox + gx * res; GY = oy + (H - 1 - gy) * res
k = (GY > -9.5) & (GY < 9.5) & (GX > -7.3)
inner = {}
for yy in np.round(GY[k], 1):
    pass
print('GT west wall inner face x (max x of wall cells per y band, excluding pilasters):', np.percentile(GX[k], 50).round(3))
