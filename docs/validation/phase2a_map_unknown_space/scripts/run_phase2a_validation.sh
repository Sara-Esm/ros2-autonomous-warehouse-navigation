#!/usr/bin/env bash
# Phase 2a validation mission: same launch + mission command as scripts/run_mission_trial.sh,
# Phase 1 station goals + map free_thresh 0.01; Nav2 parameters unchanged. Lightweight recording only:
# no gz transport probes (suspected in the v1.0.1 diagnostic crash), no CLI parameter probes.
set -Eeuo pipefail
WS="$HOME/projects/warehouse_ws"; OUT="$WS/diagnostics_phase2/run"; cd "$WS"
[[ -e "$OUT/launch.log" ]] && { echo "run exists" >&2; exit 1; }
if pgrep -x gzserver >/dev/null || pgrep -f warehouse_autonomy >/dev/null; then echo "sim running" >&2; exit 1; fi
mkdir -p "$OUT"
set +u; source /opt/ros/humble/setup.bash; source install/setup.bash; set -u
export TURTLEBOT3_MODEL=burger_cam
ts() { date +%s.%N; }; meta() { echo "$*" >> "$OUT/meta.txt"; }
{
  echo "branch: $(git branch --show-current)  head: $(git rev-parse HEAD)"
  echo "uncommitted_source_diff_sha256: $(git diff -- src | sha256sum | cut -d' ' -f1)"
  echo "untracked_stations_sha256: $(sha256sum src/warehouse_bringup/warehouse_bringup/stations.py | cut -d' ' -f1)"
  echo "installed_stations_sha256: $(sha256sum install/warehouse_bringup/lib/python3.10/site-packages/warehouse_bringup/stations.py | cut -d' ' -f1)"
  echo "nav2_params_sha256: $(sha256sum install/turtlebot3_navigation2/share/turtlebot3_navigation2/param/humble/burger_cam.yaml | cut -d' ' -f1)"
  echo "map_yaml_sha256: $(sha256sum install/warehouse_bringup/share/warehouse_bringup/maps/warehouse_map.yaml | cut -d" " -f1)"; echo "map_sha256: $(sha256sum install/warehouse_bringup/share/warehouse_bringup/maps/warehouse_map.pgm | cut -d' ' -f1)"
  echo "launch_start_wall: $(ts)"; uptime; free -h
} > "$OUT/meta.txt"
PIDS=()
setsid ros2 launch warehouse_bringup warehouse_robot_system_camera.launch.py > "$OUT/launch.log" 2>&1 &
LAUNCH_PGID=$!
( while true; do echo "=== $(ts)"; top -b -n 1 -w 220 | head -32; sleep 2; done ) > "$OUT/cpu_top.log" 2>&1 &
PIDS+=($!)
cleanup() {
  for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null || true; done
  if [[ -n "${BAG_PID:-}" ]] && kill -0 "$BAG_PID" 2>/dev/null; then kill -INT "$BAG_PID"; sleep 5; fi
  if kill -0 "$LAUNCH_PGID" 2>/dev/null; then
    kill -INT -- "-$LAUNCH_PGID" 2>/dev/null || true
    for _ in $(seq 1 30); do kill -0 "$LAUNCH_PGID" 2>/dev/null || break; sleep 1; done
    kill -TERM -- "-$LAUNCH_PGID" 2>/dev/null || true; sleep 3; kill -KILL -- "-$LAUNCH_PGID" 2>/dev/null || true
  fi
  pkill -x gzserver 2>/dev/null || true; pkill -x gzclient 2>/dev/null || true
  meta "teardown_wall: $(ts)"
}
trap cleanup EXIT
wait_active() { local n="$1" d=$(( $(date +%s) + $2 )); while (( $(date +%s) < d )); do
  timeout 8s ros2 lifecycle get "$n" 2>/dev/null | grep -q '^active' && return 0; sleep 3; done; return 1; }
wait_active /bt_navigator 300 || { meta "bt_navigator not active"; exit 1; }
meta "bt_navigator_active_wall: $(ts)"
wait_active /amcl 30 || { meta "amcl not active"; exit 1; }
TOPICS=(/clock /tf /tf_static /odom /scan /amcl_pose /plan /local_plan /cmd_vel /cmd_vel_nav
  /behavior_tree_log /map /warehouse/marker_id /warehouse/marker_detected /camera/camera_info
  /navigate_to_pose/_action/status /follow_path/_action/status /compute_path_to_pose/_action/status
  /spin/_action/status /backup/_action/status /wait/_action/status /drive_on_heading/_action/status
  /assisted_teleop/_action/status /global_costmap/costmap)
ros2 bag record --include-hidden-topics -o "$OUT/bag" "${TOPICS[@]}" > "$OUT/bag_record.log" 2>&1 &
BAG_PID=$!
sleep 8
{ echo "--- pre_mission"; uptime; free -h; } >> "$OUT/meta.txt"
meta "mission_start_wall: $(ts)"
set +e
ros2 run warehouse_bringup warehouse_autonomy --ros-args -p use_sim_time:=true > "$OUT/mission.log" 2>&1
RC=$?
set -e
meta "mission_exit_code: $RC"; meta "mission_end_wall: $(ts)"
sleep 60
meta "post_observation_end_wall: $(ts)"
{ echo "--- post"; uptime; free -h; } >> "$OUT/meta.txt"
exit "$RC"
