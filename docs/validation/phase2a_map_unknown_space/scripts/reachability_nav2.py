#!/usr/bin/env python3
"""
Static-map reachability under Nav2 conventions (supersedes reachability.py / gap_analysis.py).

Usage: reachability_nav2.py <workspace> <out_dir>

Conventions:
- Cell index as Nav2 Costmap2D::worldToMap: mx = floor((x - ox)/res), my = floor((y - oy)/res);
  PGM image row = H - 1 - my (map_server flips the image vertically).
- Classification as loaded by nav2 map_server with free_thresh 0.01 (verified: 0 -> occupied,
  254 -> free, 205 -> unknown), or the v1.0.1 load (205 -> free).
- Traversable for NavFn with allow_unknown=false: known free and distance to the nearest lethal cell
  > inscribed radius 0.14 m (cost < 253). Component label 0 = blocked, never a region.
- Static map only: cells later observed by the LiDAR (obstacle-layer clearing) are not modeled here.
"""
import collections, heapq, math, sys
import cv2, numpy as np
ws, out = sys.argv[1], sys.argv[2]
sys.path.insert(0, ws + '/src/warehouse_bringup')
from warehouse_bringup import stations as st  # noqa: E402
img = cv2.imread(ws + '/src/warehouse_bringup/maps/warehouse_map.pgm', cv2.IMREAD_UNCHANGED)
H, W = img.shape; RES, OX, OY = 0.05, -7.56, -10.8
L = []
P = lambda s='': (L.append(s), print(s))


def cell(x, y):
    mx, my = math.floor((x - OX) / RES), math.floor((y - OY) / RES)
    return H - 1 - my, mx                                  # (image row, col)


def center(r, c):
    return OX + (c + 0.5) * RES, OY + (H - 1 - r + 0.5) * RES


g_new = np.where(img == 0, 100, np.where(img == 254, 0, -1))     # free_thresh 0.01 (verified load)
g_old = np.where(img == 0, 100, 0)                                # v1.0.1 load (205 -> free)
spawn = st.world_to_map(0.0, 0.0, 0.0)[:2]
goals = {wp['station']: (wp['x'], wp['y']) for wp in st.WAYPOINTS}
P(f'map {W}x{H} @ {RES} m; spawn in map frame ({spawn[0]:+.3f},{spawn[1]:+.3f}); index convention floor() as Nav2 worldToMap')
for name, g in (('v1.0.1 load (free_thresh 0.25)', g_old), ('Phase 2a load (free_thresh 0.01)', g_new)):
    P(f'  {name}: occupied {(g == 100).sum()}, free {(g == 0).sum()}, unknown {(g == -1).sum()}')

dist = cv2.distanceTransform(255 - (g_new == 100).astype(np.uint8) * 255, cv2.DIST_L2, 5) * RES
trav = (g_new == 0) & (dist > 0.14)
n, lab = cv2.connectedComponents(trav.astype(np.uint8), connectivity=8)
P('\n## Strict policy (allow_unknown=false), static map only')


def describe(label, x, y):
    r, c = cell(x, y)
    cls = {100: 'occupied', 0: 'free', -1: 'unknown'}[int(g_new[r, c])]
    lb = lab[r, c]
    if lb == 0:
        ys, xs = np.nonzero(trav)
        k = np.argmin(np.hypot(ys - r, xs - c))
        near = lab[ys[k], xs[k]]
        P(f'  {label:15s} cell ({r},{c}) {cls}, clearance {dist[r,c]:.2f} m: BLOCKED cell (not a region); '
          f'nearest traversable cell {math.hypot(ys[k]-r, xs[k]-c)*RES:.2f} m away in region {near}')
        return near
    P(f'  {label:15s} cell ({r},{c}) {cls}, clearance {dist[r,c]:.2f} m: region {lb}')
    return lb


regions = {'spawn': describe('spawn', *spawn)}
for k, v in goals.items():
    regions[k] = describe(k, *v)
for lb in sorted(set(regions.values())):
    comp = lab == lb
    ext = bool(comp[0].any() or comp[-1].any() or comp[:, 0].any() or comp[:, -1].any())
    P(f'  region {lb}: {comp.sum()*RES*RES:.1f} m^2, contains {[k for k, v in regions.items() if v == lb]}, touches map border: {ext}')

# minimum unknown crossing between the two regions (0-1 BFS through unknown-but-clear cells)
unk = (g_new == -1) & (dist > 0.14)
north, south = regions['home_dock'], regions['inventory_scan']
if north != south:
    INF = 10 ** 9; D = np.full((H, W), INF); prev = {}; dq = collections.deque()
    for r, c in zip(*np.nonzero(lab == south)):
        D[r, c] = 0; dq.append((r, c))
    nb = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    while dq:
        r, c = dq.popleft()
        for a, b in nb:
            rr, cc = r + a, c + b
            if 0 <= rr < H and 0 <= cc < W and (trav[rr, cc] or unk[rr, cc]):
                w = 0 if trav[rr, cc] else 1
                if D[r, c] + w < D[rr, cc]:
                    D[rr, cc] = D[r, c] + w; prev[(rr, cc)] = (r, c)
                    (dq.appendleft if w == 0 else dq.append)((rr, cc))
    tgt = min(zip(*np.nonzero(lab == north)), key=lambda rc: D[rc])
    path = [tgt]
    while path[-1] in prev:
        path.append(prev[path[-1]])
    gap = [rc for rc in path if unk[rc]]
    P(f'  minimum unobserved cells separating region {south} (spawn side) from region {north} (north stations): {int(D[tgt])} '
      f'at map cell centers {[tuple(round(v, 3) for v in center(*rc)) for rc in gap]}')


def dijkstra(a, b, T):
    s, t = cell(*a), cell(*b)
    if not T[s]:
        ys, xs = np.nonzero(T); k = np.argmin(np.hypot(ys - s[0], xs - s[1])); s = (ys[k], xs[k])
    Dm = {s: 0.0}; pq = [(0.0, s)]
    while pq:
        d0, u = heapq.heappop(pq)
        if u == t:
            return d0 * RES
        if d0 > Dm[u]:
            continue
        for dr, dc, w in ((1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1), (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142)):
            v = (u[0] + dr, u[1] + dc)
            if 0 <= v[0] < H and 0 <= v[1] < W and T[v] and d0 + w < Dm.get(v, 1e18):
                Dm[v] = d0 + w; heapq.heappush(pq, (d0 + w, v))
    return float('nan')


legs = [('spawn', spawn, goals['home_dock']), ('dock->loading', goals['home_dock'], goals['loading_zone']),
        ('loading->inventory', goals['loading_zone'], goals['inventory_scan']),
        ('inventory->dispatch', goals['inventory_scan'], goals['dispatch_zone']),
        ('dispatch->dock', goals['dispatch_zone'], goals['home_dock'])]
P('  shortest route through known traversable space (grid Dijkstra; nan = no route):')
for nm, a, b in legs:
    P(f'    {nm:20s} {dijkstra(a, b, trav):6.2f} m')

vis = np.zeros((H, W, 3), np.uint8)
vis[g_new == 0] = (255, 255, 255); vis[g_new == -1] = (160, 160, 160); vis[g_new == 100] = (0, 0, 0)
palette = [(255, 220, 170), (170, 235, 170), (200, 200, 255)]
for i, lb in enumerate(sorted(set(regions.values()))):
    vis[lab == lb] = palette[i % len(palette)]
if north != south:
    for rc in gap:
        vis[rc] = (0, 0, 255)
for k, v in goals.items():
    cv2.circle(vis, cell(*v)[::-1], 3, (200, 0, 200), -1)
cv2.circle(vis, cell(*spawn)[::-1], 3, (255, 0, 0), -1)
cv2.imwrite(out + '/known_regions_nav2.png', cv2.resize(vis, (W * 3, H * 3), interpolation=cv2.INTER_NEAREST))
open(out + '/reachability_nav2.txt', 'w').write('\n'.join(L) + '\n')
