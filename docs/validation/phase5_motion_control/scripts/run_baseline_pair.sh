#!/usr/bin/env bash
# Two additional Phase 4b-configuration headless baseline missions, sequential; stop after the first failure.
cd ~/projects/warehouse_ws
for n in 2 3; do
  until awk '{exit !($1 < 2.0)}' /proc/loadavg; do sleep 5; done
  ps -eo comm | grep -qE "^(gzserver|gzclient|rviz2|component_conta)$" && { echo "host not clean before run $n"; exit 2; }
  echo "run $n start $(date -Is) load $(cut -d' ' -f1 /proc/loadavg)"
  OUT=$PWD/diagnostics_phase5/baseline_run$n timeout -k 30 1500 diagnostics_phase5/run_headless_mission.sh > diagnostics_phase5/baseline_run$n.runner.out 2>&1
  rc=$?; echo "run $n exit $rc $(date -Is)"; sleep 15
  [ $rc -ne 0 ] && { echo "stopping: run $n failed"; exit $rc; }
done
