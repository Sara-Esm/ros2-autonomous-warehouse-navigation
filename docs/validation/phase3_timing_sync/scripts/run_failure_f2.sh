#!/usr/bin/env bash
# F2 bounded live failure-path session (Phase 3 configuration). Hard-kill timeouts everywhere.
set -Eeo pipefail
WS="$HOME/projects/warehouse_ws"; OUT="$WS/diagnostics_phase3/f2"; cd "$WS"
[[ -e "$OUT/launch.log" ]] && { echo "f2 output exists" >&2; exit 1; }
if pgrep -x gzserver >/dev/null; then echo "sim running" >&2; exit 1; fi
mkdir -p "$OUT"; source /opt/ros/humble/setup.bash; source install/setup.bash; export TURTLEBOT3_MODEL=burger_cam
ts() { date +%s.%N; }; meta() { echo "$*" >> "$OUT/meta.txt"; }
echo "branch: $(git branch --show-current) head: $(git rev-parse HEAD)" > "$OUT/meta.txt"
setsid ros2 launch warehouse_bringup warehouse_robot_system_camera.launch.py > "$OUT/launch.log" 2>&1 & LP=$!
( while true; do echo "=== $(ts)"; top -b -n 1 -w 220 | head -32; sleep 2; done ) > "$OUT/cpu_top.log" 2>&1 & TOPP=$!
cleanup() {
  kill $TOPP 2>/dev/null || true
  [[ -n "${BAG:-}" ]] && kill -INT "$BAG" 2>/dev/null && sleep 5
  kill -INT -- "-$LP" 2>/dev/null || true; sleep 10; kill -KILL -- "-$LP" 2>/dev/null || true
  for p in gzserver gzclient rviz2; do pkill -x $p 2>/dev/null || true; done
  meta "teardown_wall: $(ts)"
}
trap cleanup EXIT
wait_active() { local n="$1" d=$(( $(date +%s) + $2 )); while (( $(date +%s) < d )); do
  timeout -k 3 8s ros2 lifecycle get "$n" 2>/dev/null | grep -q '^active' && return 0; sleep 3; done; return 1; }
wait_active /bt_navigator 300 || { meta "bt_navigator not active"; exit 1; }
wait_active /amcl 30 || { meta "amcl not active"; exit 1; }
ros2 bag record --include-hidden-topics -o "$OUT/bag" /clock /tf /tf_static /odom /cmd_vel /cmd_vel_nav /plan /behavior_tree_log \
  /global_costmap/costmap /navigate_to_pose/_action/status /follow_path/_action/status /compute_path_to_pose/_action/status \
  /spin/_action/status /backup/_action/status /wait/_action/status /drive_on_heading/_action/status > "$OUT/bag.log" 2>&1 & BAG=$!
sleep 8
meta "mission_start_wall: $(ts)"
set +e
timeout -k 5 420 python3 diagnostics_phase3/failure_f2_live.py "$OUT/f2_events.json" > "$OUT/f2.log" 2>&1
RC=$?
set -e
meta "f2_exit_code: $RC"; meta "mission_end_wall: $(ts)"
exit "$RC"
