#!/usr/bin/env bash
# One bounded perception bench session. Usage: run_session.sh <model> <x> <y> <yaw_rad> <out_dir> [stage_seconds]
set -o pipefail
WS="$HOME/projects/warehouse_ws"; B="$WS/diagnostics_phase4/bench"; MODEL=$1; X=$2; Y=$3; YAW=$4; OUT=$5; S=${6:-40}; GUI=${GUI:-true}
mkdir -p "$OUT"; source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"
if pgrep -x gzserver >/dev/null; then echo "sim already running" >&2; exit 1; fi
( while true; do echo "=== $(date +%s.%N)"; top -b -n 1 -w 220 | head -25; sleep 2; done ) > "$OUT/cpu_top.log" 2>&1 & TOPP=$!
setsid ros2 launch "$B/perception_bench.launch.py" model:=$MODEL gui:=$GUI x:=$X y:=$Y yaw:=$YAW > "$OUT/launch.log" 2>&1 & LP=$!
DET=""
cleanup() { kill $TOPP 2>/dev/null; [[ -n "$DET" ]] && kill -INT -- "-$DET" 2>/dev/null; sleep 2; [[ -n "$DET" ]] && kill -KILL -- "-$DET" 2>/dev/null
  kill -INT -- "-$LP" 2>/dev/null; sleep 8; kill -KILL -- "-$LP" 2>/dev/null; for p in gzserver gzclient; do pkill -x $p; done; true; }
trap cleanup EXIT
for i in $(seq 1 40); do timeout -k 2 6 ros2 topic echo --once /camera/camera_info --field header > /dev/null 2>&1 && break; sleep 3; done
sleep 5
echo "stage_camera_only_start $(date +%s.%N)" >> "$OUT/stages.txt"
timeout -k 3 $((S+20)) python3 "$B/probe.py" "$OUT/stage1_camera_only.npz" $S "$OUT/images"
setsid ros2 run warehouse_perception aruco_station_detector --ros-args -p use_sim_time:=true -p debug_window:=false > "$OUT/det_nowin.log" 2>&1 & DET=$!
sleep 5; echo "stage_detector_start $(date +%s.%N)" >> "$OUT/stages.txt"
timeout -k 3 $((S+20)) python3 "$B/probe.py" "$OUT/stage2_detector.npz" $S
kill -INT -- "-$DET" 2>/dev/null; sleep 2; kill -KILL -- "-$DET" 2>/dev/null
setsid ros2 run warehouse_perception aruco_station_detector --ros-args -p use_sim_time:=true -p debug_window:=true > "$OUT/det_win.log" 2>&1 & DET=$!
sleep 5; echo "stage_detector_window_start $(date +%s.%N)" >> "$OUT/stages.txt"
timeout -k 3 $((S+20)) python3 "$B/probe.py" "$OUT/stage3_detector_window.npz" $S
echo "end $(date +%s.%N)" >> "$OUT/stages.txt"
