#!/usr/bin/env bash
# Phase 2b mapping session: Gazebo + robot, stock TurtleBot3 Cartographer (real simulated LiDAR),
# scripted collision-aware coverage drive. Writes only to diagnostics_phase2b/mapping/.
set -Eeuo pipefail
WS="$HOME/projects/warehouse_ws"; OUT="$WS/diagnostics_phase2b/mapping"; cd "$WS"
[[ -e "$OUT/sim.log" ]] && { echo "mapping output exists" >&2; exit 1; }
if pgrep -x gzserver >/dev/null; then echo "sim running" >&2; exit 1; fi
mkdir -p "$OUT"
set +u; source /opt/ros/humble/setup.bash; source install/setup.bash; set -u
export TURTLEBOT3_MODEL=burger_cam
ts() { date +%s.%N; }; meta() { echo "$*" >> "$OUT/meta.txt"; }
{ echo "branch: $(git branch --show-current) head: $(git rev-parse HEAD)"
  echo "cartographer_config_sha256: $(sha256sum install/turtlebot3_cartographer/share/turtlebot3_cartographer/config/turtlebot3_lds_2d.lua | cut -d' ' -f1)"
  echo "start_wall: $(ts)"; uptime; } > "$OUT/meta.txt"
PG=()
setsid ros2 launch warehouse_bringup warehouse_simulation_camera.launch.py > "$OUT/sim.log" 2>&1 & PG+=($!)
cleanup() {
  [[ -n "${BAG:-}" ]] && kill -INT "$BAG" 2>/dev/null && sleep 5
  for g in "${PG[@]}"; do kill -INT -- "-$g" 2>/dev/null || true; done
  sleep 8
  for g in "${PG[@]}"; do kill -KILL -- "-$g" 2>/dev/null || true; done
  pkill -x gzserver 2>/dev/null || true; pkill -x gzclient 2>/dev/null || true
  meta "teardown_wall: $(ts)"
}
trap cleanup EXIT
# wait for the robot to publish odometry and scans
for i in $(seq 1 60); do timeout 5 ros2 topic echo --once /scan --field header >/dev/null 2>&1 && break; sleep 3; done
meta "sensors_up_wall: $(ts)"
setsid ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true use_rviz:=false > "$OUT/cartographer.log" 2>&1 & PG+=($!)
for i in $(seq 1 60); do timeout 8 ros2 topic echo --once /map --field info >/dev/null 2>&1 && break; sleep 3; done
meta "cartographer_map_up_wall: $(ts)"
ros2 bag record -o "$OUT/bag" /scan /odom /tf /tf_static /clock > "$OUT/bag.log" 2>&1 & BAG=$!
sleep 5
meta "drive_start_wall: $(ts)"
set +e
python3 diagnostics_phase2b/route/coverage_drive.py "$OUT/drive.log" > "$OUT/drive.stdout" 2>&1
RC=$?
set -e
meta "drive_exit_code: $RC"; meta "drive_end_wall: $(ts)"
sleep 5   # let Cartographer integrate the last scans
timeout 60 ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}" > "$OUT/finish_trajectory.txt" 2>&1 || true
timeout 120 ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '$OUT/warehouse_map_v2.pbstream', include_unfinished_submaps: true}" > "$OUT/write_state.txt" 2>&1 || true
sleep 3
timeout 120 ros2 run nav2_map_server map_saver_cli -f "$OUT/warehouse_map_v2" --ros-args -p use_sim_time:=true -p save_map_timeout:=30.0 > "$OUT/map_saver.txt" 2>&1 || true
meta "saved_wall: $(ts)"
ls -la "$OUT" >> "$OUT/meta.txt"
exit "$RC"
