#!/usr/bin/env bash
# Replay a recorded headless mission's /scan,/tf,/tf_static,/clock into one standalone local costmap per
# configuration, each on its own DDS domain (the standalone node always names itself /costmap/costmap).
# Usage: run_local_unknown_replay.sh <bag_dir> <out_dir>
set -Eeuo pipefail
WS="$HOME/projects/warehouse_ws"; BAG=$1; OUT=$2; cd "$WS"; mkdir -p "$OUT"
set +u; source /opt/ros/humble/setup.bash; set -u; export ROS_LOCALHOST_ONLY=1
DUR=$(python3 -c "import re;m=open('$BAG/../meta.txt').read();print(int(float(re.search(r'mission_end_wall: ([\d.]+)',m).group(1))-float(re.search(r'mission_start_wall: ([\d.]+)',m).group(1)))+40)")
PG=()
cleanup() { for p in "${PG[@]}"; do kill -INT -- "-$p" 2>/dev/null || true; done; sleep 3; for p in "${PG[@]}"; do kill -KILL -- "-$p" 2>/dev/null || true; done; }
trap cleanup EXIT
dom=191
for k in lc_current lc_track lc_track_inf; do
  ROS_DOMAIN_ID=$dom setsid ros2 run nav2_costmap_2d nav2_costmap_2d --ros-args --params-file "$WS/diagnostics_phase5/local_unknown_$k.yaml" > "$OUT/$k.log" 2>&1 & PG+=($!)
  dom=$((dom+1))
done
sleep 6; dom=191
for k in lc_current lc_track lc_track_inf; do   # configure now; activation needs TF, so it follows the replay start
  ROS_DOMAIN_ID=$dom timeout -k 3 30 ros2 lifecycle set /costmap/costmap configure > /dev/null
  ROS_DOMAIN_ID=$dom setsid python3 diagnostics_phase5/local_unknown_probe.py "$OUT/$k.npz" $((DUR+15)) $k > "$OUT/$k.probe.log" 2>&1 & PG+=($!)
  dom=$((dom+1))
done
sleep 2; dom=191; PL=()
for k in lc_current lc_track lc_track_inf; do
  ROS_DOMAIN_ID=$dom timeout -k 5 $((DUR+60)) ros2 bag play "$BAG" --topics /clock /tf /tf_static /scan > "$OUT/$k.play.log" 2>&1 & PL+=($!)
  dom=$((dom+1))
done
sleep 3; dom=191
for k in lc_current lc_track lc_track_inf; do
  ROS_DOMAIN_ID=$dom timeout -k 3 40 ros2 lifecycle set /costmap/costmap activate > /dev/null || echo "$k activate failed" | tee -a "$OUT/nodes.txt"
  echo "$k domain $dom: $(ROS_DOMAIN_ID=$dom timeout -k 3 10 ros2 lifecycle get /costmap/costmap) track_unknown_space=$(ROS_DOMAIN_ID=$dom timeout -k 3 10 ros2 param get /costmap/costmap track_unknown_space | tail -1) inf_is_valid=$(ROS_DOMAIN_ID=$dom timeout -k 3 10 ros2 param get /costmap/costmap obstacle_layer.scan.inf_is_valid 2>&1 | tail -1)" | tee -a "$OUT/nodes.txt"
  dom=$((dom+1))
done
for p in "${PL[@]}"; do wait $p || true; done
sleep 20; cat "$OUT"/*.probe.log
