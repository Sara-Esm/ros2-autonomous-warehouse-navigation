#!/usr/bin/env python3
"""Offline per-frame cost of the detector pipeline on recorded frames (same calls as aruco_station_detector).

Usage: detector_cost.py <images_dir> [...]
Times: Image msg -> cv2 (cv_bridge imgmsg_to_cv2 bgr8), BGR->gray, detectMarkers (DICT_5X5_50). 200 reps/frame set.
"""
import glob, sys, time
import numpy as np, cv2
from cv_bridge import CvBridge
sys.path.insert(0, '/home/owl/projects/warehouse_ws/src/warehouse_perception')
from warehouse_perception.aruco_station_detector import make_detector
det = make_detector(cv2.aruco.DICT_5X5_50); br = CvBridge()
for d in sys.argv[1:]:
    frames = [np.load(f) for f in sorted(glob.glob(d + '/img_*.npy'))]
    msgs = [br.cv2_to_imgmsg(f[:, :, ::-1].copy(), encoding='rgb8') for f in frames]
    tc, tg, td, found = [], [], [], 0
    for rep in range(200):
        for m in msgs:
            t0 = time.perf_counter(); fr = br.imgmsg_to_cv2(m, desired_encoding='bgr8'); t1 = time.perf_counter()
            g = cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY); t2 = time.perf_counter()
            c, ids, rej = det(g); t3 = time.perf_counter()
            tc.append(t1 - t0); tg.append(t2 - t1); td.append(t3 - t2); found += ids is not None
    n = len(tc)
    print(f'{d}: {len(frames)} frames {frames[0].shape}; conversion p50 {np.median(tc)*1e3:.3f} ms, gray {np.median(tg)*1e3:.3f} ms, '
          f'detectMarkers p50 {np.median(td)*1e3:.2f} ms p99 {np.percentile(td,99)*1e3:.2f} ms; detected in {found/n:.0%} of frames')
