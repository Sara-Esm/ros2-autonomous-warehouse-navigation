#!/usr/bin/env bash
# One bounded controller-experiment session. Usage: run_variant.sh <name> <variant_yaml>
# Writes the variant into the INSTALLED burger_cam.yaml for this session only and always restores it.
set -Eeuo pipefail
WS="$HOME/projects/warehouse_ws"; NAME=$1; VY=$2; OUT="$WS/diagnostics_phase5/variants/$NAME"; cd "$WS"
INST=install/turtlebot3_navigation2/share/turtlebot3_navigation2/param/humble/burger_cam.yaml
SRC=src/turtlebot3/turtlebot3_navigation2/param/humble/burger_cam.yaml
[[ -e "$OUT/launch.log" ]] && { echo "exists" >&2; exit 1; }
if pgrep -x gzserver >/dev/null; then echo "sim running" >&2; exit 1; fi
cmp -s "$INST" "$SRC" || { echo "installed params differ from source before start" >&2; exit 1; }
mkdir -p "$OUT"; set +u; source /opt/ros/humble/setup.bash; source install/setup.bash; set -u; export TURTLEBOT3_MODEL=burger_cam
ts() { date +%s.%N; }; meta() { echo "$*" >> "$OUT/meta.txt"; }
cp "$VY" "$INST"; echo "variant: $NAME route ${ROUTE:-north} sha256 $(sha256sum "$VY" | cut -c1-64)" > "$OUT/meta.txt"; cp "$VY" "$OUT/params.yaml"
setsid ros2 launch warehouse_bringup warehouse_robot_system_camera.launch.py gui:=false use_rviz:=false debug_window:=false > "$OUT/launch.log" 2>&1 & LP=$!
( while true; do echo "=== $(ts)"; top -b -n 1 -w 220 | head -25; sleep 2; done ) > "$OUT/cpu_top.log" 2>&1 & TOPP=$!
BAG=""
cleanup() { kill $TOPP 2>/dev/null || true; [[ -n "$BAG" ]] && { kill -INT $BAG 2>/dev/null || true; sleep 5; }
  # Teardown by tracked process group (setsid leader $LP = PGID), never by process-name matching.
  meta "launch_pgid: $LP members_before_teardown: $(pgrep -g "$LP" | tr '\n' ' ')"
  kill -INT -- "-$LP" 2>/dev/null || true
  for _ in $(seq 1 30); do pgrep -g "$LP" >/dev/null || break; sleep 1; done
  if pgrep -g "$LP" >/dev/null; then kill -TERM -- "-$LP" 2>/dev/null || true; sleep 3; fi
  if pgrep -g "$LP" >/dev/null; then meta "SIGKILL survivors: $(pgrep -g "$LP" | tr '\n' ' ')"; kill -KILL -- "-$LP" 2>/dev/null || true; sleep 1; fi
  meta "process_group_empty_after_teardown: $(pgrep -g "$LP" >/dev/null && echo NO || echo yes)"
  cp "$SRC" "$INST"; cmp -s "$INST" "$SRC" && meta "params restored" || meta "PARAM RESTORE FAILED"; meta "teardown_wall: $(ts)"; }
trap cleanup EXIT
wait_active() { local n="$1" d=$(( $(date +%s) + $2 )); while (( $(date +%s) < d )); do
  timeout -k 3 8s ros2 lifecycle get "$n" 2>/dev/null | grep -q '^active' && return 0; sleep 3; done; return 1; }
wait_active /bt_navigator 300 || { meta "bt_navigator not active"; exit 1; }
wait_active /amcl 30 || { meta "amcl not active"; exit 1; }
ros2 bag record --include-hidden-topics -o "$OUT/bag" /clock /tf /tf_static /odom /cmd_vel /cmd_vel_nav /plan /amcl_pose /behavior_tree_log \
  /navigate_to_pose/_action/status /follow_path/_action/status /compute_path_to_pose/_action/status /spin/_action/status \
  /backup/_action/status /wait/_action/status /drive_on_heading/_action/status /global_costmap/costmap > "$OUT/bag.log" 2>&1 & BAG=$!
sleep 8
{ for p in FollowPath.acc_lim_theta FollowPath.PathAlign.forward_point_distance general_goal_checker.yaw_goal_tolerance FollowPath.acc_lim_x; do
    echo "/controller_server $p: $(timeout -k 3 15 ros2 param get /controller_server $p 2>&1 | tail -1)"; done
  for p in max_accel max_decel max_velocity; do echo "/velocity_smoother $p: $(timeout -k 3 15 ros2 param get /velocity_smoother $p 2>&1 | tail -1)"; done; } > "$OUT/runtime_params.txt"
meta "mission_start_wall: $(ts)"; set +e
ROUTE=${ROUTE:-north} timeout -k 5 1500 python3 diagnostics_phase5/route_node.py > "$OUT/mission.log" 2>&1; RC=$?; set -e
meta "mission_exit_code: $RC"; meta "mission_end_wall: $(ts)"; exit $RC
