#!/usr/bin/env python3
"""Summarize a bench session: per stage image / camera_info / marker rates (sim and wall), RTF, image age, CPU.

Usage: analyze.py <session_dir> [...]
image age = sim clock at receipt - image header stamp (sim seconds; transport + queueing in sim time).
"""
import re, sys
import numpy as np
def cpu(sd, a, b):
    txt = open(sd + '/cpu_top.log', errors='ignore').read().split('=== ')[1:]
    acc = {}; busy = []; mem = []
    for blk in txt:
        t = float(blk.split('\n', 1)[0])
        if not a <= t <= b: continue
        m = re.search(r'%Cpu\(s\):\s+([\d.]+) us,\s+([\d.]+) sy.*?([\d.]+) id', blk)
        if m: busy.append(100 - float(m.group(3)))
        m = re.search(r'MiB Mem :\s+[\d.]+ total,\s+[\d.]+ free,\s+([\d.]+) used', blk)
        if m: mem.append(float(m.group(1)))
        for l in blk.splitlines():
            f = l.split()
            if len(f) >= 12 and f[0].isdigit():
                name = f[11]
                if name.startswith('python3'): name = 'python3'
                acc.setdefault(name, []).append(float(f[8]))
    n = max(1, len([1 for blk in txt if a <= float(blk.split('\n', 1)[0]) <= b]))
    top = sorted(((k, sum(v) / n) for k, v in acc.items()), key=lambda x: -x[1])[:5]
    return np.median(busy) if busy else float('nan'), np.median(mem) if mem else float('nan'), top
for sd in sys.argv[1:]:
    st = {l.split()[0]: float(l.split()[1]) for l in open(sd + '/stages.txt')}
    print(f'== {sd.rstrip("/").split("/")[-1]}')
    for stage, key in (('stage1_camera_only', 'stage_camera_only_start'), ('stage2_detector', 'stage_detector_start'), ('stage3_detector_window', 'stage_detector_window_start')):
        z = np.load(f'{sd}/{stage}.npz'); clk = z['clock']
        if len(clk) < 2: print(f'  {stage}: no clock'); continue
        wall = clk[-1, 0] - clk[0, 0]; sim = clk[-1, 1] - clk[0, 1]; rtf = sim / wall
        img = z['img']; info = z['info']; mk = z['marker']
        def rate(x, col):
            return (len(x) - 1) / (x[-1, col] - x[0, col]) if len(x) > 2 else 0.0
        simstamps = img[:, 2] if len(img) else np.array([])
        r_img_sim = (len(img) - 1) / (simstamps[-1] - simstamps[0]) if len(img) > 2 else 0
        age = (img[:, 1] - img[:, 2]) * 1e3 if len(img) else np.array([np.nan])
        busy, mem, top = cpu(sd, z['clock'][0, 0], z['clock'][-1, 0])
        mks = sorted(set(mk[:, 2].astype(int))) if len(mk) else []
        print(f'  {stage:24s} RTF {rtf:.3f} | image {r_img_sim:5.2f} Hz sim {rate(img,0):5.2f} Hz wall ({len(img)}) | camera_info {(len(info)-1)/sim if len(info)>1 else 0:5.2f} Hz sim'
              f' | marker msgs {len(mk)/sim if sim else 0:5.2f} Hz sim ids {mks} | age p50 {np.median(age):.0f} ms sim max {np.max(age):.0f}'
              f' | CPU busy {busy:.0f}% mem {mem:.0f} MiB | ' + ', '.join(f'{k} {v:.0f}%' for k, v in top))
