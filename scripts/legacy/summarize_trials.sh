#!/usr/bin/env bash
# Summarize the archived pilot and comparable baseline Trials 02-04.
set -Eeuo pipefail
cd "$HOME/projects/warehouse_ws" || exit 1
python3 - <<'PY'
from pathlib import Path
import re

root = Path('audit_logs')
print('WAREHOUSE BASELINE TRIALS')
print('Trial 01 is PILOT (uncontrolled 81-min launch-to-mission delay); 02-04 are the comparison cohort.')
print('Success counts are per trial, not a statistical reliability guarantee.\n')
headers = ['Trial','Group','Tasks','Failed','Duration(s)','BT warns','Ctrl warns','Plan warns','Follow aborts','Progress errors','RTF pre']
rows=[]
for trial in range(1,5):
    k=f'{trial:02d}'
    mission_path=root/f'mission_baseline_trial{k}.log'
    launch_path=root/f'launch_trial{k}.log'
    meta_path=root/f'meta_trial{k}.txt'
    mission=mission_path.read_text(errors='replace') if mission_path.exists() else ''
    launch=launch_path.read_text(errors='replace') if launch_path.exists() else ''
    meta=meta_path.read_text(errors='replace') if meta_path.exists() else ''
    def match(pattern, text):
        m=re.search(pattern,text,re.M)
        return m.group(1) if m else '?'
    success=match(r'Successful tasks:\s*(\d+)', mission)
    failed=match(r'Failed tasks:\s*(\d+)', mission)
    first=re.search(r'\[(\d+(?:\.\d+)?)\].*MISSION STEP 1/5',mission)
    last=re.search(r'\[(\d+(?:\.\d+)?)\].*(?:WAREHOUSE MISSION (?:COMPLETE|FINISHED)|MISSION HALTED)',mission)
    duration=f'{float(last.group(1))-float(first.group(1)):.1f}' if first and last else '?'
    bt=str(launch.count('Behavior Tree tick rate')) if launch else '?'
    ctrl=str(launch.count('Control loop missed its desired rate')) if launch else '?'
    plan=str(launch.count('Planner loop missed its desired rate')) if launch else '?'
    abort=str(len(re.findall(r'\[follow_path\].*Aborting handle',launch))) if launch else '?'
    progress=str(launch.count('Failed to make progress')) if launch else '?'
    rtf=match(r'^rtf_pre_mission:\s*(.*)$',meta) if meta else '?'
    rows.append([k,'PILOT' if trial==1 else 'BASE', success,failed,duration,bt,ctrl,plan,abort,progress,rtf])
widths=[max(len(str(x)) for x in col) for col in zip(headers,*rows)]
def fmt(row):
    return '  '.join(str(s).ljust(w) for s,w in zip(row,widths))
print(fmt(headers))
print('  '.join('-'*w for w in widths))
for row in rows: print(fmt(row))
print('\nAdditional recovery signals in launch logs (message counts, not failed mission counts):')
for trial in range(1,5):
    k=f'{trial:02d}'
    p=root/f'launch_trial{k}.log'
    if not p.exists():
        print(f'Trial {k}: launch log unavailable')
        continue
    log=p.read_text(errors='replace')
    keys={'ack timeouts':'Timed out while waiting for action server to acknowledge',
          'clear service timeouts':'Node timed out while executing service call',
          'spin failed':'spin failed',
          'Goal succeeded':'Goal succeeded'}
    print(f'Trial {k}: '+', '.join(f'{name}={log.count(token)}' for name,token in keys.items()))
print('\nVerify trial starts were from fresh simulation launches, with unchanged configuration and comparable initial pose.')
PY
