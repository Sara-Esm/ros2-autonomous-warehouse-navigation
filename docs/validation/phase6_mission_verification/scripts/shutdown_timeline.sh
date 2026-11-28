#!/usr/bin/env bash
# Shutdown-only timeline: headless system up (Nav2 active, detector running, no mission), SIGINT to
# `ros2 launch` only, then record on ONE wall clock (time.time / date +%s.%N):
#   signal time; every launch log line's receipt time (signals sent, escalations, exits);
#   each process-group member's exit time (polled by exact PID every 50 ms); group-empty time.
set -Eeuo pipefail
WS="$HOME/projects/warehouse_ws"; OUT=$1; cd "$WS"; mkdir -p "$OUT"
set +u; source /opt/ros/humble/setup.bash; source install/setup.bash; set -u; export TURTLEBOT3_MODEL=burger_cam
setsid python3 scripts/sigdfl_exec.py ros2 launch warehouse_bringup warehouse_robot_system_camera.launch.py \
  gui:=false use_rviz:=false debug_window:=false 2>&1 | python3 -u diagnostics_phase6/line_stamper.py > "$OUT/launch_stamped.log" &
sleep 1; LP=$(pgrep -f "^python3 scripts/sigdfl_exec.py|^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch warehouse_bringup" | head -1)
LP=$(ps -o pgid= -p "$LP" | tr -d ' ')
echo "launch_pgid $LP" > "$OUT/timeline.txt"
final() { if pgrep -g "$LP" >/dev/null; then echo "FORCED KILL by probe: $(pgrep -g "$LP" | tr '\n' ' ')" >> "$OUT/timeline.txt"; kill -KILL -- "-$LP" 2>/dev/null || true; fi; }
trap final EXIT
d=$(( $(date +%s) + 300 )); until timeout -k 3 8 ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q '^active'; do (( $(date +%s) < d )) || exit 1; sleep 3; done
sleep 25
declare -A NAME; for p in $(pgrep -g "$LP"); do NAME[$p]=$(cat /proc/$p/comm 2>/dev/null); echo "member $p ${NAME[$p]}" >> "$OUT/timeline.txt"; done
T0=$(date +%s.%N); kill -INT "$LP"; echo "signal SIGINT to ros2 launch (pid $LP) at $T0" >> "$OUT/timeline.txt"
declare -A GONE
for i in $(seq 1 600); do
  for p in "${!NAME[@]}"; do
    if [[ -z "${GONE[$p]:-}" ]] && ! kill -0 "$p" 2>/dev/null; then GONE[$p]=$(date +%s.%N); fi
    if [[ -z "${GONE[$p]:-}" ]] && grep -q '^State:.*Z' /proc/$p/status 2>/dev/null; then GONE[$p]=$(date +%s.%N); fi
  done
  if ! pgrep -g "$LP" >/dev/null; then echo "group_empty $(date +%s.%N)" >> "$OUT/timeline.txt"; break; fi
  sleep 0.05
done
for p in "${!NAME[@]}"; do echo "exit $p ${NAME[$p]} ${GONE[$p]:-STILL_ALIVE}" >> "$OUT/timeline.txt"; done
