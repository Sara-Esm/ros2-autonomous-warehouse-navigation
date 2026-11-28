#!/usr/bin/env bash
# Phase 7 repeatability campaign: H2, G1, H3, G2, G3 (H1 = Phase 6 headless validation). Sequential, quiet host,
# every run preserved; stop after the first non-zero mission exit so it can be reported before continuing.
cd ~/projects/warehouse_ws
for spec in H2:headless G1:graphical H3:headless G2:graphical G3:graphical; do
  name=${spec%%:*}; mode=${spec##*:}
  until awk '{exit !($1 < 2.0)}' /proc/loadavg; do sleep 5; done
  ps -eo comm | grep -qE "^(gzserver|gzclient|rviz2|component_conta)$" && { echo "host not clean before $name"; exit 2; }
  echo "$name ($mode) start $(date -Is) load $(cut -d' ' -f1 /proc/loadavg)"
  MODE=$mode OUT=$PWD/diagnostics_phase7/$name timeout -k 30 1500 diagnostics_phase7/run_mission.sh > diagnostics_phase7/$name.runner.out 2>&1
  rc=$?; echo "$name exit $rc $(date -Is); $(grep -E 'shutdown_seconds|group_empty' diagnostics_phase7/$name/meta.txt 2>/dev/null | tr '\n' ' ')"
  sleep 15
  [ $rc -ne 0 ] && { echo "stopping after $name (exit $rc)"; exit $rc; }
done
echo "campaign complete"
