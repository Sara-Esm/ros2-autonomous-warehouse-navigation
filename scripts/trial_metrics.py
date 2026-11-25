#!/usr/bin/env python3
"""
Extract comparable metrics from saved trial logs (read-only).

Usage:
    scripts/trial_metrics.py [--markdown] TRIAL [TRIAL ...]
    scripts/trial_metrics.py --all

For trial NN it reads audit_logs/launch_trialNN.log, audit_logs/meta_trialNN.txt
and the mission log (mission_trialNN.log, mission_tuning_trialNN.log,
mission_validation_trialNN.log or mission_baseline_trialNN.log). Every value is
counted from the logs; '-' means the log or field is missing.
"""

import argparse
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parent.parent / 'audit_logs'
MISSION_PATTERNS = ('mission_trial{}.log', 'mission_tuning_trial{}.log',
                    'mission_validation_trial{}.log', 'mission_baseline_trial{}.log')
END_RE = re.compile(r'\[(\d+\.\d+)\].*(WAREHOUSE MISSION (?:COMPLETE|FINISHED)|MISSION HALTED)')
START_RE = re.compile(r'\[(\d+\.\d+)\].*MISSION STEP 1/5')

COLUMNS = [
    ('trial', 'Trial'), ('kind', 'Kind'), ('gui', 'GUI'), ('commit', 'Commit'),
    ('tasks_ok', 'Tasks OK'), ('tasks_failed', 'Failed'), ('nav_failures', 'Nav failures'),
    ('retries', 'Retries'), ('duration_s', 'Mission s'), ('rtf', 'RTF'),
    ('ctrl_missed', 'Ctrl rate missed'), ('no_valid_traj', 'No valid traj'),
    ('no_progress', 'No progress'), ('follow_path_aborts', 'follow_path aborts'),
    ('planner_fail', 'Plan failures'), ('recoveries', 'Spin+backup'),
    ('initpose_warn', 'Init-pose warnings'),
]


def read(path):
    return path.read_text(errors='replace') if path.exists() else None


def first(pattern, text, cast=str):
    if text is None:
        return None
    m = re.search(pattern, text, re.M)
    return cast(m.group(1)) if m else None


def metrics(trial):
    t = f'{int(trial):02d}'
    launch = read(ROOT / f'launch_trial{t}.log')
    meta = read(ROOT / f'meta_trial{t}.txt')
    mission = None
    for pattern in MISSION_PATTERNS:
        mission = read(ROOT / pattern.format(t))
        if mission is not None:
            break

    kind = first(r'^kind:\s*(\S+)', meta) or ('pilot' if t == '01' else None)
    gui = 'off' if kind and 'gui-off' in kind else ('on' if kind else None)
    row = {
        'trial': t, 'kind': kind, 'gui': gui,
        'commit': (first(r'^commit:\s*(\w{7})', meta)),
        'rtf': first(r'^rtf_pre_mission:\s*([\d.]+)', meta),
    }
    if mission is not None:
        ok = re.findall(r'Successful tasks:\s*(\d+)', mission)
        failed = re.findall(r'Failed tasks:\s*(\d+)', mission)
        row['tasks_ok'] = ok[-1] if ok else None
        row['tasks_failed'] = failed[-1] if failed else None
        row['nav_failures'] = str(mission.count('Navigation FAILED'))
        row['retries'] = str(mission.count('RETRYING navigation'))
        start, end = START_RE.search(mission), END_RE.search(mission)
        if start and end:
            row['duration_s'] = f'{float(end.group(1)) - float(start.group(1)):.0f}'
    if launch is not None:
        row.update({
            'ctrl_missed': launch.count('Control loop missed its desired rate'),
            'no_valid_traj': launch.count('No valid trajectories'),
            'no_progress': launch.count('Failed to make progress'),
            'follow_path_aborts': len(re.findall(r'\[follow_path\].*Aborting handle', launch)),
            'planner_fail': launch.count('failed to create plan'),
            'recoveries': launch.count('Running spin') + launch.count('Running backup'),
            'initpose_warn': launch.count('Please set the initial pose'),
        })
    return {k: ('-' if row.get(k) is None else str(row[k])) for k, _ in COLUMNS}


def all_trials():
    found = set()
    for path in ROOT.glob('*trial*'):
        m = re.search(r'trial(\d{2})', path.name)
        if m and 'failed_readiness' not in path.name:
            found.add(m.group(1))
    return sorted(found)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('trials', nargs='*')
    parser.add_argument('--all', action='store_true')
    parser.add_argument('--markdown', action='store_true')
    args = parser.parse_args()
    trials = all_trials() if args.all else args.trials
    if not trials:
        parser.error('give trial numbers or --all')
    rows = [metrics(t) for t in trials]
    headers = [h for _, h in COLUMNS]
    table = [[r[k] for k, _ in COLUMNS] for r in rows]
    if args.markdown:
        print('| ' + ' | '.join(headers) + ' |')
        print('|' + '---|' * len(headers))
        for r in table:
            print('| ' + ' | '.join(r) + ' |')
    else:
        widths = [max(len(x) for x in col) for col in zip(headers, *table)]
        for r in [headers] + table:
            print('  '.join(c.ljust(w) for c, w in zip(r, widths)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
