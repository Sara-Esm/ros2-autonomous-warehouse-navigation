#!/usr/bin/env python3
"""Reduce the station survey to map-frame goals and validate them against simulation truth.

Usage: survey_analysis.py <survey.json> <out_txt> <out_goals_json>
Evaluation-only fit (map_quality.py on warehouse_map_v2): world = R(0.71 deg) * map + (-0.009, +0.040).
"""
import json, math, sys
import numpy as np
S = json.load(open(sys.argv[1])); out, gout = sys.argv[2], sys.argv[3]
TH, TX, TY = math.radians(0.71), -0.009, 0.040
to_world = lambda x, y: (math.cos(TH) * x - math.sin(TH) * y + TX, math.sin(TH) * x + math.cos(TH) * y + TY)
L = []; goals = {}
P = lambda s='': (L.append(s), print(s))
P('| Station | Surveyed map goal (x, y, yaw) | Sample spread (pos / yaw) | AMCL 1-sigma (x, y, yaw) | Placement error (truth vs intended) | '
  'Surveyed goal -> world vs intended (evaluation fit) | Localization residual at survey (eval fit) |')
P('|---|---|---|---|---|---|---|')
for name, d in S.items():
    s = d['samples']
    x = np.array([v['map_x'] for v in s]); y = np.array([v['map_y'] for v in s])
    a = np.radians([v['map_yaw_deg'] for v in s]); yaw = math.degrees(math.atan2(np.sin(a).mean(), np.cos(a).mean()))
    gx, gy = float(x.mean()), float(y.mean())
    spread = float(np.max(np.hypot(x - gx, y - gy))); yspread = float(np.max(np.abs((np.degrees(a) - yaw + 180) % 360 - 180)))
    ix, iy, iyaw = d['intended_world']
    tx = np.mean([v['truth_x'] for v in s]); ty = np.mean([v['truth_y'] for v in s]); tyaw = np.mean([v['truth_yaw_deg'] for v in s])
    place = math.hypot(tx - ix, ty - iy)
    wx, wy = to_world(gx, gy); wyaw = yaw + math.degrees(TH)
    gerr = math.hypot(wx - ix, wy - iy); gyerr = (wyaw - iyaw + 180) % 360 - 180
    lres = math.hypot(wx - tx, wy - ty)
    sig = [np.mean([v[k] for v in s]) for k in ('amcl_sigma_x', 'amcl_sigma_y', 'amcl_sigma_yaw_deg')]
    P(f'| {name} | ({gx:+.3f}, {gy:+.3f}, {yaw:+.2f}°) | {100*spread:.1f} cm / {yspread:.1f}° | ({sig[0]:.2f} m, {sig[1]:.2f} m, {sig[2]:.1f}°) | '
      f'{100*place:.1f} cm, {tyaw-iyaw:+.2f}° | {100*gerr:.1f} cm, {gyerr:+.2f}° | {100*lres:.1f} cm |')
    goals[name] = {'x': round(gx, 3), 'y': round(gy, 3), 'yaw': round(yaw, 2), 'spread_m': round(spread, 4),
                   'yaw_spread_deg': round(yspread, 2), 'amcl_sigma': [round(v, 3) for v in sig],
                   'placement_error_m': round(place, 4), 'eval_goal_error_m': round(gerr, 4), 'eval_goal_yaw_error_deg': round(gyerr, 2)}
json.dump(goals, open(gout, 'w'), indent=1)
open(out, 'w').write('\n'.join(L) + '\n')
