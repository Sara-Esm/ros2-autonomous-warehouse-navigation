#!/usr/bin/env bash
# One instrumented diagnostic mission. Same launch + mission command as
# scripts/run_mission_trial.sh; adds passive recorders only. Writes only to
# diagnostics_20261125/run/. Does not touch audit_logs/, params, map or code.
set -Eeuo pipefail
WS="$HOME/projects/warehouse_ws"
OUT="$WS/diagnostics_20261125/run"
cd "$WS"
if [[ -e "$OUT/launch.log" ]]; then echo "run already exists" >&2; exit 1; fi
if pgrep -x gzserver >/dev/null || pgrep -f warehouse_autonomy >/dev/null; then
  echo "another simulation/mission is running" >&2; exit 1; fi
mkdir -p "$OUT"
set +u; source /opt/ros/humble/setup.bash; source install/setup.bash; set -u
export TURTLEBOT3_MODEL=burger_cam
ts() { date +%s.%N; }
meta() { echo "$*" >> "$OUT/meta.txt"; }
{
  echo "commit: $(git rev-parse HEAD) ($(git describe --tags))"
  echo "nav2_params_sha256: $(sha256sum install/turtlebot3_navigation2/share/turtlebot3_navigation2/param/humble/burger_cam.yaml | cut -d' ' -f1)"
  echo "launch_start_wall: $(ts)"; uptime; free -h
} > "$OUT/meta.txt"

PIDS=()
setsid ros2 launch warehouse_bringup warehouse_robot_system_camera.launch.py > "$OUT/launch.log" 2>&1 &
LAUNCH_PGID=$!

# CPU sampler (whole run).
( while true; do echo "=== $(ts)"; top -b -n 1 -w 220 | head -32; sleep 2; done ) > "$OUT/cpu_top.log" 2>&1 &
PIDS+=($!)

cleanup() {
  for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null || true; done
  if [[ -n "${BAG_PID:-}" ]] && kill -0 "$BAG_PID" 2>/dev/null; then kill -INT "$BAG_PID"; sleep 5; fi
  if kill -0 "$LAUNCH_PGID" 2>/dev/null; then
    kill -INT -- "-$LAUNCH_PGID" 2>/dev/null || true
    for _ in $(seq 1 30); do kill -0 "$LAUNCH_PGID" 2>/dev/null || break; sleep 1; done
    kill -TERM -- "-$LAUNCH_PGID" 2>/dev/null || true; sleep 3
    kill -KILL -- "-$LAUNCH_PGID" 2>/dev/null || true
  fi
  pkill -x gzserver 2>/dev/null || true; pkill -x gzclient 2>/dev/null || true
  meta "teardown_wall: $(ts)"
}
trap cleanup EXIT

wait_active() {
  local node="$1" deadline=$(( $(date +%s) + $2 ))
  while (( $(date +%s) < deadline )); do
    if timeout 8s ros2 lifecycle get "$node" 2>/dev/null | grep -q '^active'; then return 0; fi
    sleep 3
  done; return 1
}
wait_active /bt_navigator 300 || { meta "bt_navigator not active"; exit 1; }
meta "bt_navigator_active_wall: $(ts)"
wait_active /amcl 30 || { meta "amcl not active"; exit 1; }
timeout 60s ros2 topic echo --once /amcl_pose --field pose.pose > "$OUT/amcl_initial.txt" || true

# Gazebo RTF stream and ground-truth pose sampler.
( gz stats -p 2>&1 | while IFS= read -r l; do echo "$(ts) $l"; done ) > "$OUT/gz_stats.log" &
PIDS+=($!)
( while true; do echo "$(ts) $(timeout 3 gz model -m turtlebot3 -p 2>&1 | tr '\n' ' ')"; sleep 1; done ) > "$OUT/gz_truth_pose.log" &
PIDS+=($!)

# Rosbag of navigation-relevant topics (no images).
TOPICS=(/clock /tf /tf_static /odom /scan /amcl_pose /initialpose /plan /local_plan
  /cmd_vel /cmd_vel_nav /behavior_tree_log /map /global_costmap/costmap
  /local_costmap/costmap /warehouse/marker_id /warehouse/marker_detected
  /camera/camera_info /rosout
  /navigate_to_pose/_action/status /follow_path/_action/status
  /compute_path_to_pose/_action/status /spin/_action/status /backup/_action/status
  /wait/_action/status /drive_on_heading/_action/status /assisted_teleop/_action/status)
ros2 bag record -o "$OUT/bag" "${TOPICS[@]}" > "$OUT/bag_record.log" 2>&1 &
BAG_PID=$!
sleep 6

# Configuration snapshots (effective parameters, QoS, node list).
ros2 node list > "$OUT/nodes.txt" 2>&1 || true
mkdir -p "$OUT/params"
for n in controller_server planner_server bt_navigator amcl velocity_smoother behavior_server \
         global_costmap/global_costmap local_costmap/local_costmap robot_state_publisher \
         aruco_station_detector map_server smoother_server; do
  timeout 20 ros2 param dump "/$n" > "$OUT/params/$(echo $n | tr / _).yaml" 2>&1 || true
done
for t in /scan /odom /cmd_vel /cmd_vel_nav /camera/image_raw /warehouse/marker_id /amcl_pose /map /tf /clock /plan; do
  echo "##### $t"; timeout 15 ros2 topic info -v "$t" 2>&1
done > "$OUT/qos_topic_info.txt"
timeout 15 ros2 run tf2_ros tf2_echo map odom > "$OUT/tf_map_odom_initial.txt" 2>&1 || true
{ echo "--- pre_mission"; uptime; free -h; } >> "$OUT/meta.txt"

meta "mission_start_wall: $(ts)"
set +e
ros2 run warehouse_bringup warehouse_autonomy --ros-args -p use_sim_time:=true > "$OUT/mission.log" 2>&1
RC=$?
set -e
meta "mission_exit_code: $RC"
meta "mission_end_wall: $(ts)"
# Observe 60 s after completion: does the robot keep moving?
sleep 60
meta "post_observation_end_wall: $(ts)"
{ echo "--- post"; uptime; free -h; } >> "$OUT/meta.txt"
exit "$RC"
