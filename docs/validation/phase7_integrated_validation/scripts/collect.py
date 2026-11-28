#!/usr/bin/env python3
"""Collect per-run headline numbers from each run's analysis/ files into one table (no recomputation)."""
import re, sys
RUNS = [('H1', 'diagnostics_phase6/validation'), ('H2', 'diagnostics_phase7/H2'), ('H3', 'diagnostics_phase7/H3'),
        ('G1', 'diagnostics_phase7/G1'), ('G2', 'diagnostics_phase7/G2'), ('G3', 'diagnostics_phase7/G3')]
def rd(run, f):
    try: return open(f'{run}/analysis/{f}').read()
    except FileNotFoundError: return ''
def g(pat, s, cast=str, default='?'):
    m = re.search(pat, s); return cast(m.group(1)) if m else default
rows = []
for lab, run in RUNS:
    meta = open(f'{run}/meta.txt').read(); p1 = rd(run, 'phase1_results.txt'); per = rd(run, 'perception.txt'); tim = rd(run, 'timing.txt')
    mot = rd(run, 'motion.txt'); arr = rd(run, 'arrivals.txt'); acc = rd(run, 'angular_accel.txt'); ver = rd(run, 'verification_shutdown.txt')
    unk = rd(run, 'unknown_traversal.txt'); lut = rd(run, 'live_unknown_timing.txt'); clr = rd(run, 'clearance_mechanisms.txt')
    r = {'run': lab, 'mode': g(r'mode: (\w+)', meta, default='headless'), 'exit': g(r'mission_exit_code: (\d+)', meta),
         'wall_s': g(r'mission (\d+) s wall', per), 'rtf': g(r'RTF ([\d.]+)', per),
         'verified': g(r'\| (\d/\d) verified', per), 'cam_hz': g(r'camera_info: ([\d.]+) Hz sim', per),
         'pos_max': g(r'position error: max ([\d.]+) m', p1), 'pos_mean': g(r'position error: max [\d.]+ m, mean ([\d.]+)', p1),
         'head_settled_mean': g(r'settled \|heading\| mean ([\d.]+) deg', arr), 'head_settled_max': g(r'settled \|heading\| mean [\d.]+ deg, max ([\d.]+)', arr),
         'dx_range': g(r'dx range ([-+\d.]+ \.\. [-+\d.]+)', arr),
         'settle_s': g(r'settling after goal success \(sim s\): ([\d., ]+)', ver), 'evid_age_ms': g(r'image age at verification \(ms sim\): ([\d, ]+)', ver),
         'final_check': 'passed' if 'FINAL CHECK: robot stationary' in ver else 'NOT SEEN',
         'ack_to': g(r'ack timeouts (\d+)', tim), 'orphans': g(r'late-accepted \(orphaned\) goals after an ack timeout: (\d+)', tim),
         'overlap_s': g(r'executing at once: ([\d.]+) s', tim),
         'ctrl_hz': g(r'/cmd_vel_nav\s+([\d.]+) Hz sim', tim), 'smooth_hz': g(r'/cmd_vel\s+([\d.]+) Hz sim', tim),
         'no_progress': g(r'no_progress (\d+)', p1), 'fp_abort': g(r'fp_abort (\d+)', p1),
         'recov': '/'.join(g(rf'{k} (\d+)', p1) for k in ('spin', 'backup', 'wait')),
         'reversals': g(r'cruise ([\d.]+), slow', mot), 'rotation_deg': g(r'rotation (\d+) deg\n', mot + '\n'),
         'acc_p95_ls02': g(r'ls_0\.2s p95 ([\d.]+)', acc), 'acc_max_ls02': g(r'ls_0\.2s p95 [\d.]+ max ([\d.]+)', acc),
         'pinch_centre': g(r'PINCH: centre ([\d.]+)', clr), 'pinch_fp_min': g(r'window min footprint ([\d.]+)', clr),
         'plan_clear': g(r'plan clearance ([\d.]+) m', clr), 'off_plan': g(r'robot off plan ([\d.]+ cm \(\w+)', clr),
         'amcl_err_cm': g(r'AMCL error ([\d.]+) cm', clr),
         'live_unk': g(r'executed on live-unknown \(sim-stamp ordering\): ([\d.]+) m', lut), 'unk_resid': g(r'residual \(unknown in this AND the next snapshot\): ([\d.]+) m', lut),
         'min_clear_all': min([float(x) for x in re.findall(r'^\| [A-Z][^|]*\|(?:[^|]*\|){9}\s*([\d.]+)', unk, re.M)] or [float('nan')]),
         'shutdown_s': g(r'shutdown_seconds: ([\d.]+)', meta), 'group_empty': g(r'process_group_empty_after_teardown: (\w+)', meta),
         'exits': '; '.join(f'{n}: {e}' for n, e in sorted(set(re.findall(r'exit: ([\w.]+)-\d\s+(finished cleanly|died code -?\d+)', ver)))) }
    rows.append(r)
keys = list(rows[0].keys())
print('| ' + ' | '.join(keys) + ' |'); print('|' + '---|' * len(keys))
for r in rows: print('| ' + ' | '.join(str(r[k]) for k in keys) + ' |')
