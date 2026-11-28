#!/usr/bin/env bash
# Integrated per-run analysis with the established scripts (definitions unchanged). Usage: analyze_run.sh <label> <run_dir>
set -uo pipefail
L=$1; R=$2; WS="$HOME/projects/warehouse_ws"; cd "$WS"; A="$R/analysis"; mkdir -p "$A"
set +u; source /opt/ros/humble/setup.bash; set -u
X="numpy version|warnings.warn|rosbag2_storage"
E="python3 diagnostics_phase2b/eval_wrapper.py"
python3 diagnostics_phase4/perception_metrics.py "$L=$R" 2>&1 | grep -vE "$X" > "$A/perception.txt"
python3 diagnostics_phase3/timing_metrics.py "$L=$R" 2>&1 | grep -vE "$X" > "$A/timing.txt"
python3 diagnostics_phase5/motion_metrics.py "$L=$R" 2>&1 | grep -vE "$X" > "$A/motion.txt"
python3 diagnostics_phase5/session_eval.py "$L=$R" 2>&1 | grep -vE "$X" > "$A/arrivals.txt"
python3 diagnostics_phase6/angular_accel_methods.py "$L=$R" 2>&1 | grep -vE "$X" > "$A/angular_accel.txt"
python3 diagnostics_phase6/phase6_metrics.py "$L=$R" 2>&1 | grep -vE "$X" > "$A/verification_shutdown.txt"
$E docs/validation/phase1_station_frames/scripts/phase1_evaluate.py "$R" diagnostics_20261125/run . > "$A/phase1_results.txt" 2>/dev/null
$E diagnostics_phase2b/unknown_traversal_v2.py "$R" . "$L" "$A/unknown_traversal.txt" > /dev/null 2>&1
$E diagnostics_phase4/live_unknown_timing.py "$R" . 2>&1 | grep -vE "$X" > "$A/live_unknown_timing.txt"
$E diagnostics_phase5/clearance_mechanisms.py "$L=$R" 2>&1 | grep -vE "$X" > "$A/clearance_mechanisms.txt"
echo "analyzed $L"
