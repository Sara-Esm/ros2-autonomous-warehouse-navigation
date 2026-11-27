#!/usr/bin/env bash
# Usage: run_perturb.sh <model> <out_dir> [seconds_per_pose]   (headless gzserver + detector, bounded)
WS="$HOME/projects/warehouse_ws"; B="$WS/diagnostics_phase4/bench"; MODEL=$1; OUT=$2; D=${3:-5}; SET=${4:-original44}
mkdir -p "$OUT"; source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"
if pgrep -x gzserver >/dev/null; then echo "sim already running" >&2; exit 1; fi
setsid ros2 launch "$B/perception_bench.launch.py" model:=$MODEL gui:=false x:=0 y:=0 yaw:=0 > "$OUT/launch.log" 2>&1 & LP=$!
setsid ros2 run warehouse_perception aruco_station_detector --ros-args -p use_sim_time:=true -p debug_window:=false > "$OUT/det.log" 2>&1 & DET=$!
cleanup() { kill -INT -- "-$DET" 2>/dev/null; kill -INT -- "-$LP" 2>/dev/null; sleep 8; kill -KILL -- "-$DET" -- "-$LP" 2>/dev/null; pkill -x gzserver; true; }
trap cleanup EXIT
for i in $(seq 1 40); do timeout -k 2 6 ros2 topic echo --once /camera/camera_info --field header > /dev/null 2>&1 && break; sleep 3; done
timeout -k 5 3000 python3 "$B/perturb.py" "$OUT/perturb.json" $D $SET > "$OUT/perturb.log" 2>&1
