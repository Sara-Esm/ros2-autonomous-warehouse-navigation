#!/usr/bin/env python3
"""Summarize the +-0.25 m detection-envelope campaign (perturb.py grid25). Usage: envelope_summary.py <perturb.json>"""
import json, sys
from collections import defaultdict
R = json.load(open(sys.argv[1]))
bad_pose = [r for r in R if r['pose_err_m'] > 0.02 or abs(((r['true_pose'][2] - ({'home_dock': 90, 'loading_zone': 90}.get(r['station'], -90) + r['dyaw'])) + 180) % 360 - 180) > 2]
print(f'poses {len(R)}; poses whose read-back differs from the command (>2 cm or >2 deg, e.g. contact with geometry): {len(bad_pose)}')
for r in bad_pose: print(f"   {r['station']} dx{r['dx']:+.3f} dy{r['dy']:+.3f} da{r['dyaw']:+.1f} err {r['pose_err_m']} true {r['true_pose']}")
F = sum(r['frames'] for r in R); D = sum(r['detected'] for r in R); W = sum(r['wrong'] for r in R)
print(f'frames {F}; detected {D}; missed {F-D}; wrong-ID messages {W}')
def rate(r): return r['detected'] / r['frames'] if r['frames'] else float('nan')
print('per station: ' + '; '.join(f"{s} {sum(r['detected'] for r in R if r['station']==s)}/{sum(r['frames'] for r in R if r['station']==s)}" for s in dict.fromkeys(r['station'] for r in R)))
poor = [r for r in R if rate(r) < 1.0]
print(f'poses with any missed frame: {len(poor)}; with < 90% detection: {sum(rate(r) < 0.9 for r in R)}; with 0 detections: {sum(r["detected"] == 0 for r in R)}')
for r in sorted(poor, key=lambda r: (r['station'], r['dx'], r['dy'], r['dyaw'])):
    print(f"   {r['station']:15s} dx{r['dx']:+.3f} dy{r['dy']:+.3f} da{r['dyaw']:+5.1f}  {r['detected']}/{r['frames']}  wrong {r['wrong']}  range {r['marker_range_m']:.2f} m bearing {r['marker_bearing_deg']:+.1f} deg")
print('\ndetection by dx (along heading, + = toward marker), all stations and headings:')
for dx in sorted({r['dx'] for r in R}):
    s = [r for r in R if r['dx'] == dx]; print(f"   dx {dx:+.3f}: {sum(x['detected'] for x in s)}/{sum(x['frames'] for x in s)}  poses failing any frame {sum(rate(x)<1 for x in s)}/{len(s)}  range {min(x['marker_range_m'] for x in s):.2f}-{max(x['marker_range_m'] for x in s):.2f} m")
print('detection by |bearing| band:')
for lo, hi in ((0, 10), (10, 20), (20, 30), (30, 40), (40, 60)):
    s = [r for r in R if lo <= abs(r['marker_bearing_deg']) < hi]
    if s: print(f"   {lo:2d}-{hi:2d} deg: {len(s)} poses, {sum(x['detected'] for x in s)}/{sum(x['frames'] for x in s)}")
print(f"tested extremes: max |bearing| {max(abs(r['marker_bearing_deg']) for r in R):.1f} deg, range {min(r['marker_range_m'] for r in R):.2f}-{max(r['marker_range_m'] for r in R):.2f} m, true offset from goal up to {max(r['true_offset_from_goal_m'] for r in R):.3f} m")
