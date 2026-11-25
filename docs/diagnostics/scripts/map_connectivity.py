#!/usr/bin/env python3
"""Is the map's free space (205 loaded as FREE) connected to the outside of the building?"""
import sys, numpy as np, cv2
ws, out = sys.argv[1], sys.argv[2]
img = cv2.imread(ws + '/src/warehouse_bringup/maps/warehouse_map.pgm', cv2.IMREAD_UNCHANGED)
H, W = img.shape; res = 0.05; ox, oy = -7.56, -10.8
occ = ((1 - img / 255) > 0.65).astype(np.uint8)
for rad in (0.0, 0.14):
    k = int(np.ceil(rad / res)); ker = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2*k+1, 2*k+1)) if k else np.ones((1,1),np.uint8)
    blocked = cv2.dilate(occ, ker)
    n, lab = cv2.connectedComponents((1 - blocked).astype(np.uint8), connectivity=8)
    def L(x, y): return lab[int(round(H-1-(y-oy)/res)), int(round((x-ox)/res))]
    s = L(0, 0)
    comp = lab == s
    rows, cols = np.nonzero(comp)
    xs = ox + cols*res; ys = oy + (H-1-rows)*res
    outside = (xs < -6.99) | (xs > 6.99) | (ys < -10.45) | (ys > 10.45)
    border = comp[0,:].any() or comp[-1,:].any() or comp[:,0].any() or comp[:,-1].any()
    print(f'inflate {rad:.2f} m: spawn component touches image border={border}; cells beyond true outer walls in it: {outside.sum()} '
          f'(x range {xs.min():.2f}..{xs.max():.2f}, y range {ys.min():.2f}..{ys.max():.2f})')
    for nm,(x,y) in {'dock':(-3.678,9.3),'loading':(1.803,9.3),'inventory':(1.801,-9.0),'dispatch':(-3.63,-9.3)}.items():
        print(f'   {nm} reachable: {L(x,y)==s}')
    if rad == 0.14:
        vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR); vis[comp] = (vis[comp]*0.6 + np.array([0,120,0])*0.4).astype(np.uint8)
        m = np.zeros_like(img, bool); m[rows[outside], cols[outside]] = True
        vis[m] = (0,0,255)
        cv2.imwrite(out + '/map_free_outside_walls.png', cv2.resize(vis, (W*3, H*3), interpolation=cv2.INTER_NEAREST))
