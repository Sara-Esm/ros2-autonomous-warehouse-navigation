#!/usr/bin/env python3
"""Run a Phase 1/2a analysis script unchanged, supplying the EVALUATION-ONLY v2 frame fit.

Usage: eval_wrapper.py <script> [args...]
The operational stations module no longer contains a ground-truth transform. For evaluation, this
wrapper attaches map_to_world/world_to_map for warehouse_map_v2 (map_quality.py fit:
world = R(0.71 deg) * map + (-0.009, +0.040)) before running the script.
"""
import math, os, runpy, sys
ws = os.path.expanduser('~/projects/warehouse_ws')
sys.path.insert(0, ws + '/src/warehouse_bringup')
import warehouse_bringup.stations as st  # noqa: E402
TH, TX, TY = 0.71, -0.009, 0.040


def map_to_world(x, y, yaw):
    r = math.radians(TH)
    return (math.cos(r) * x - math.sin(r) * y + TX, math.sin(r) * x + math.cos(r) * y + TY, st.normalize_deg(yaw + TH))


def world_to_map(x, y, yaw):
    r = math.radians(TH); dx, dy = x - TX, y - TY
    return (math.cos(r) * dx + math.sin(r) * dy, -math.sin(r) * dx + math.cos(r) * dy, st.normalize_deg(yaw - TH))


st.map_to_world, st.world_to_map = map_to_world, world_to_map
script = sys.argv[1]; sys.argv = sys.argv[1:]
runpy.run_path(script, run_name='__main__')
