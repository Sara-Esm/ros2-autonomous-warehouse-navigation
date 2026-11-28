#!/usr/bin/env bash
# Shutdown experiment: headless system up (Nav2 active, no mission), then stop it one of two ways and time it.
# Usage: shutdown_probe.sh <group|launch> <out_dir>
#   group : SIGINT to the whole launch process group at once (what the Phase 3-5 runners did)
#   launch: SIGINT to the `ros2 launch` process only; launch forwards it and escalates on its own
# Records per-process exit lines (launch.log), group membership over time, and a hard 60 s bound.
set -Eeuo pipefail
WS="$HOME/projects/warehouse_ws"; MODE=$1; OUT=$2; cd "$WS"; mkdir -p "$OUT"
set +u; source /opt/ros/humble/setup.bash; source install/setup.bash; set -u; export TURTLEBOT3_MODEL=burger_cam
setsid python3 "$WS/diagnostics_phase6/sigdfl_exec.py" ros2 launch warehouse_bringup warehouse_robot_system_camera.launch.py gui:=false use_rviz:=false debug_window:=false > "$OUT/launch.log" 2>&1 & LP=$!
echo "mode $MODE launch_pgid $LP launcher sigdfl_exec SigIgn $(awk '/SigIgn/{print $2}' /proc/$LP/status)" > "$OUT/meta.txt"
final() { if pgrep -g "$LP" >/dev/null; then echo "FORCED KILL of survivors: $(pgrep -g "$LP" | tr '\n' ' ')" >> "$OUT/meta.txt"; kill -KILL -- "-$LP" 2>/dev/null || true; fi; }
trap final EXIT
d=$(( $(date +%s) + 300 )); until timeout -k 3 8 ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q '^active'; do (( $(date +%s) < d )) || { echo "nav2 not active" >> "$OUT/meta.txt"; exit 1; }; sleep 3; done
sleep 20   # detector (t=55 s) and everything running
for p in $(pgrep -g "$LP"); do echo "member $p $(cat /proc/$p/comm 2>/dev/null)"; done >> "$OUT/meta.txt"
T0=$(date +%s.%N); echo "signal_wall $T0" >> "$OUT/meta.txt"
if [ "$MODE" = group ]; then kill -INT -- "-$LP"; else kill -INT "$LP"; fi
for i in $(seq 1 600); do
  pgrep -g "$LP" >/dev/null || { echo "group_empty_after_s $(echo "$(date +%s.%N) - $T0" | bc)" >> "$OUT/meta.txt"; break; }
  [ $((i % 20)) -eq 0 ] && echo "t+$((i/10))s remaining: $(for p in $(pgrep -g "$LP"); do printf '%s ' "$(cat /proc/$p/comm 2>/dev/null)"; done)" >> "$OUT/meta.txt"
  sleep 0.1
done
