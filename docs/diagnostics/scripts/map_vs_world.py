#!/usr/bin/env python3
"""Compare the saved SLAM map against Gazebo world collision geometry.

Slices every collision mesh in small_warehouse.world with horizontal planes
(LiDAR height and a robot-body band), rasterizes the slice onto the map grid,
then measures alignment (best rigid offset), coverage, phantom/missing
obstacles, and clearance at station goals. Read-only: writes to out dir only.
"""
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import cv2

WS = Path(sys.argv[1])
OUT = Path(sys.argv[2])
OUT.mkdir(parents=True, exist_ok=True)
MODELS = WS / 'src/warehouse_gazebo/models'
WORLD = WS / 'src/warehouse_gazebo/worlds/small_warehouse.world'
MAPYAML = WS / 'src/warehouse_bringup/maps/warehouse_map.yaml'
MAPPGM = WS / 'src/warehouse_bringup/maps/warehouse_map.pgm'
LIDAR_Z = 0.171 + 0.010   # base_scan in model (0.171) + spawn settles ~0.01
SLICE_ZS = [LIDAR_Z]
BODY_ZS = np.arange(0.02, 0.20, 0.02)   # burger body height ~0.19 m

STATIONS = {
    'dock': (-3.678, 9.3, 90), 'loading': (1.803, 9.3, 90),
    'inventory': (1.801, -9.0, -90), 'dispatch': (-3.630, -9.3, -90),
    'spawn': (0.0, 0.0, 0),
}
MARKERS = {'dock': (-3.678265, 9.97), 'loading': (1.802768, 9.97),
           'inventory': (1.800570, -9.97), 'dispatch': (-3.629830, -9.97)}


def rpy_mat(x, y, z, r, p, yw):
    cr, sr, cp, sp, cy, sy = map(lambda f: f, (math.cos(r), math.sin(r), math.cos(p),
                                               math.sin(p), math.cos(yw), math.sin(yw)))
    R = np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
                  [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
                  [-sp, cp*sr, cp*cr]])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T


def parse_pose(text):
    v = [float(t) for t in (text or '0 0 0 0 0 0').split()]
    return rpy_mat(*v)


def strip_ns(root):
    for el in root.iter():
        if '}' in el.tag:
            el.tag = el.tag.split('}', 1)[1]
    return root


def load_dae(path):
    root = strip_ns(ET.parse(path).getroot())
    unit = root.find('.//asset/unit')
    scale = float(unit.get('meter')) if unit is not None else 1.0
    geoms = {}
    for g in root.iter('geometry'):
        mesh = g.find('mesh')
        if mesh is None:
            continue
        srcs = {s.get('id'): np.array(s.find('float_array').text.split(), float)
                for s in mesh.findall('source') if s.find('float_array') is not None}
        verts = mesh.find('vertices')
        pos_id = verts.find("input[@semantic='POSITION']").get('source')[1:]
        P = srcs[pos_id].reshape(-1, 3)
        tris = []
        for prim in list(mesh.findall('triangles')) + list(mesh.findall('polylist')):
            inputs = prim.findall('input')
            stride = max(int(i.get('offset')) for i in inputs) + 1
            voff = [int(i.get('offset')) for i in inputs if i.get('semantic') == 'VERTEX'][0]
            idx = np.array(prim.find('p').text.split(), int).reshape(-1, stride)[:, voff]
            if prim.tag == 'polylist':
                vc = np.array(prim.find('vcount').text.split(), int)
                k = 0
                for n in vc:
                    poly = idx[k:k+n]
                    for j in range(1, n-1):
                        tris.append([poly[0], poly[j], poly[j+1]])
                    k += n
            else:
                tris.extend(idx.reshape(-1, 3).tolist())
        geoms[g.get('id')] = (P, np.array(tris, int))
    out = []
    def walk(node, T):
        m = node.find('matrix')
        if m is not None:
            T = T @ np.array(m.text.split(), float).reshape(4, 4)
        for ig in node.findall('instance_geometry'):
            gid = ig.get('url')[1:]
            if gid in geoms:
                P, tri = geoms[gid]
                Ph = np.c_[P, np.ones(len(P))] @ T.T
                out.append((Ph[:, :3] * scale, tri))
        for c in node.findall('node'):
            walk(c, T)
    for vs in root.iter('visual_scene'):
        for n in vs.findall('node'):
            walk(n, np.eye(4))
    return out


def world_models():
    root = ET.parse(WORLD).getroot()
    res = []
    for m in root.find('world').findall('model'):
        inc = m.find('include')
        if inc is None:
            continue
        uri = inc.find('uri').text.replace('model://', '')
        res.append((m.get('name'), uri, parse_pose(m.find('pose').text)))
    return res


def model_collisions(uri):
    sdf = strip_ns(ET.parse(MODELS / uri / 'model.sdf').getroot())
    res = []
    for link in sdf.iter('link'):
        Tl = parse_pose(link.findtext('pose'))
        for col in link.findall('collision'):
            Tc = parse_pose(col.findtext('pose'))
            mesh = col.find('geometry/mesh')
            if mesh is None:
                continue
            path = MODELS / mesh.findtext('uri').replace('model://', '').replace(uri + '/', uri + '/', 1)
            sc = np.array((mesh.findtext('scale') or '1 1 1').split(), float)
            res.append((path, Tl @ Tc, sc))
    return res


def slice_tris(V, tri, z):
    a, b, c = V[tri[:, 0]], V[tri[:, 1]], V[tri[:, 2]]
    segs = []
    for (p, q, r) in ((a, b, c),):
        pass
    zs = np.stack([a[:, 2], b[:, 2], c[:, 2]], 1)
    hit = (zs.min(1) <= z) & (zs.max(1) >= z)
    for i in np.nonzero(hit)[0]:
        pts = []
        tv = (a[i], b[i], c[i])
        for u, v in ((0, 1), (1, 2), (2, 0)):
            pu, pv = tv[u], tv[v]
            if (pu[2] - z) * (pv[2] - z) < 0:
                t = (z - pu[2]) / (pv[2] - pu[2])
                pts.append(pu + t * (pv - pu))
            elif pu[2] == z:
                pts.append(pu)
        if len(pts) >= 2:
            segs.append((pts[0][:2], pts[1][:2]))
    return segs


def main():
    meta = dict(re.findall(r'(\w+):\s*(.+)', MAPYAML.read_text()))
    res = float(meta['resolution'])
    ox, oy = [float(v) for v in meta['origin'].strip('[] ').split(',')[:2]]
    img = cv2.imread(str(MAPPGM), cv2.IMREAD_UNCHANGED)
    H, W = img.shape
    p = (255 - img.astype(float)) / 255.0
    occ = p > 0.65
    free = p < 0.25
    unk = ~(occ | free)

    def w2px(x, y):
        return (x - ox) / res, H - 1 - (y - oy) / res   # col, row (row 0 = top)

    gt_lidar = np.zeros((H, W), np.uint8)
    gt_body = np.zeros((H, W), np.uint8)
    inventory = []
    for name, uri, Tw in world_models():
        for path, Tlc, sc in model_collisions(uri):
            if not path.exists():
                inventory.append((name, 'MISSING MESH', str(path)))
                continue
            for V, tri in load_dae(path):
                V = V * sc
                Vh = np.c_[V, np.ones(len(V))] @ (Tw @ Tlc).T
                V = Vh[:, :3]
                inventory.append((name, f'z {V[:,2].min():.2f}..{V[:,2].max():.2f}',
                                  f'x {V[:,0].min():.2f}..{V[:,0].max():.2f} y {V[:,1].min():.2f}..{V[:,1].max():.2f}'))
                for z, target in [(z, gt_lidar) for z in SLICE_ZS] + [(z, gt_body) for z in BODY_ZS]:
                    for s0, s1 in slice_tris(V, tri, z):
                        c0, r0 = w2px(*s0)
                        c1, r1 = w2px(*s1)
                        cv2.line(target, (int(round(c0)), int(round(r0))),
                                 (int(round(c1)), int(round(r1))), 255, 1)
    with open(OUT / 'world_collision_inventory.txt', 'w') as f:
        for row in inventory:
            f.write(' | '.join(row) + '\n')

    # Alignment: search rigid offset of GT that best matches map occupied cells.
    occ_u8 = occ.astype(np.uint8)
    dist_to_map = cv2.distanceTransform(255 - occ_u8 * 255, cv2.DIST_L2, 5) * res
    ys, xs = np.nonzero(gt_lidar)
    gx = ox + xs * res
    gy = oy + (H - 1 - ys) * res
    best = None
    for dth in np.radians(np.arange(-3, 3.01, 0.25)):
        c, s = math.cos(dth), math.sin(dth)
        rx = c * gx - s * gy
        ry = s * gx + c * gy
        for dx in np.arange(-0.5, 0.501, 0.05):
            for dy in np.arange(-0.5, 0.501, 0.05):
                cc, rr = w2px(rx + dx, ry + dy)
                cc = np.round(cc).astype(int)
                rr = np.round(rr).astype(int)
                ok = (cc >= 0) & (cc < W) & (rr >= 0) & (rr < H)
                d = np.full(len(cc), 1.0)
                d[ok] = np.minimum(dist_to_map[rr[ok], cc[ok]], 1.0)
                score = np.median(d)
                m = d.mean()
                if best is None or (score, m) < best[0]:
                    best = ((score, m), dx, dy, math.degrees(dth))
    (med0, mean0), dx, dy, dth = best

    # Unshifted statistics.
    cc, rr = w2px(gx, gy)
    cc = np.round(cc).astype(int)
    rr = np.round(rr).astype(int)
    ok = (cc >= 0) & (cc < W) & (rr >= 0) & (rr < H)
    d_gt_to_map = np.minimum(dist_to_map[rr[ok], cc[ok]], 2.0)
    dist_to_gt = cv2.distanceTransform(255 - gt_body, cv2.DIST_L2, 5) * res
    my, mx = np.nonzero(occ)
    d_map_to_gt = dist_to_gt[my, mx]
    gt_body_bool = gt_body > 0
    lines = []
    P = lines.append
    P(f'map: {W}x{H} px, res {res} m, origin ({ox}, {oy}), extent x[{ox:.2f},{ox+W*res:.2f}] y[{oy:.2f},{oy+H*res:.2f}]')
    P(f'map cells: occupied {occ.sum()}, free {free.sum()}, unknown {unk.sum()} ({100*unk.mean():.1f}% unknown)')
    P(f'GT LiDAR-height slice cells: {(gt_lidar>0).sum()}; GT body-band cells: {gt_body_bool.sum()}')
    P(f'best rigid offset (map = R*world + t): dx={dx:+.2f} m dy={dy:+.2f} m dtheta={dth:+.2f} deg; median residual {med0:.3f} m mean {mean0:.3f} m')
    for thr in (0.05, 0.10, 0.20, 0.50):
        P(f'  GT slice cells with a map obstacle within {thr:.2f} m (no offset): {100*np.mean(d_gt_to_map <= thr):.1f}%')
    for thr in (0.05, 0.10, 0.20, 0.50):
        P(f'  map occupied cells within {thr:.2f} m of GT body geometry: {100*np.mean(d_map_to_gt <= thr):.1f}%')
    phantom = occ & (dist_to_gt > 0.25)
    P(f'phantom map obstacles (>0.25 m from any GT geometry): {phantom.sum()} cells')
    gt_marked_free = gt_body_bool & free
    P(f'GT body-band geometry cells that the map says are FREE: {gt_marked_free.sum()} cells')
    # Unknown cells inside the building interior that are reachable (adjacent to free).
    unk_near_free = unk & (cv2.dilate(free.astype(np.uint8), np.ones((3, 3))) > 0)
    P(f'unknown cells bordering free space (holes/frontiers): {unk_near_free.sum()}')

    # Station clearance against both GT and map.
    for k, (x, y, yaw) in STATIONS.items():
        c, r = w2px(x, y)
        c, r = int(round(c)), int(round(r))
        P(f'station {k:10s} ({x:+.3f},{y:+.3f}) map cell value={"occ" if occ[r,c] else "free" if free[r,c] else "unknown"}'
          f' clearance to map obstacle={dist_to_map[r,c]:.2f} m, to GT geometry={dist_to_gt[r,c]:.2f} m')
        if k in MARKERS:
            mxw, myw = MARKERS[k]
            P(f'    marker at ({mxw:+.3f},{myw:+.3f}): offset from goal dx={mxw-x:+.3f} dy={myw-y:+.3f} '
              f'dist={math.hypot(mxw-x, myw-y):.3f} m; bearing from goal heading={math.degrees(math.atan2(myw-y, mxw-x))-yaw:+.1f} deg')
            # wall position behind marker in GT and map along goal heading
            for label, D in (('GT', dist_to_gt), ('map', dist_to_map)):
                step = 0.025
                t = 0.0
                hy = math.sin(math.radians(yaw))
                while t < 2.0:
                    cc2, rr2 = w2px(x, y + hy * t)
                    cc2, rr2 = int(round(cc2)), int(round(rr2))
                    if not (0 <= rr2 < H and 0 <= cc2 < W):
                        break
                    if D[rr2, cc2] < 0.03:
                        break
                    t += step
                P(f'    first {label} obstacle straight ahead of goal: {t:.3f} m (marker plane at {abs(myw-y):.3f} m)')
    (OUT / 'map_vs_world.txt').write_text('\n'.join(lines) + '\n')
    print('\n'.join(lines))

    # Overlay image: map grey, GT body red, GT lidar green, phantom blue.
    vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    vis[gt_body_bool] = (0, 0, 255)
    vis[gt_lidar > 0] = (0, 200, 0)
    vis[phantom] = (255, 0, 0)
    for k, (x, y, yaw) in STATIONS.items():
        c, r = w2px(x, y)
        cv2.circle(vis, (int(c), int(r)), 3, (0, 165, 255), -1)
    for k, (x, y) in MARKERS.items():
        c, r = w2px(x, y)
        cv2.drawMarker(vis, (int(c), int(r)), (255, 0, 255), cv2.MARKER_CROSS, 6)
    vis = cv2.resize(vis, (W * 3, H * 3), interpolation=cv2.INTER_NEAREST)
    cv2.imwrite(str(OUT / 'map_vs_world_overlay.png'), vis)
    np.save(OUT / 'gt_lidar.npy', gt_lidar)
    np.save(OUT / 'gt_body.npy', gt_body)


if __name__ == '__main__':
    main()
